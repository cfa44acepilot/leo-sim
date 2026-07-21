/*****************************************************************************
  filename vk_renderer.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    The Vulkan 1.4 implementation behind vk_renderer.hpp: dynamic rendering (no
    VkRenderPass objects) and synchronization2 throughout.

    Two paths share every pipeline and the whole scene recording: the windowed
    present path and the headless capture path. That sharing is deliberate -- it
    is what makes a --screenshot a true witness of what the window shows, rather
    than a second renderer that could quietly drift away from it.
 *****************************************************************************/

#include "vk_renderer.hpp"

#include <algorithm>  /* std::clamp/max: extents, capacities            */
#include <cstdio>     /* std::fprintf: setup failures name themselves   */
#include <cstring>    /* std::memcpy: the persistently mapped uploads   */
#include <fstream>    /* reading compiled SPIR-V off disk               */
#include <vector>     /* Vulkan's enumerate-into-a-vector idiom         */

/* GLFW must know Vulkan exists before it is included, so it can declare the
   surface entry points -- hence the define rather than a plain include. */
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>  /* window + surface (windowed path only)       */

#include "imgui.h"              /* the panels drawn over the scene       */
#include "imgui_impl_glfw.h"    /* input plumbing for those panels       */
#include "imgui_impl_vulkan.h"  /* its second, color-only render pass    */

#include "stb_image.h"  /* decode the Earth day-map (JPEG)              */

/* CMake passes the real asset directories in. The fallbacks keep a hand-built
   binary running from the repo root instead of failing to find its shaders.
   These are genuine preprocessor constants (string literals pasted at compile
   time), which is the one job the style still reserves for #define. */
#ifndef MONITOR_SHADER_DIR
#define MONITOR_SHADER_DIR "shaders"
#endif
#ifndef MONITOR_TEXTURE_DIR
#define MONITOR_TEXTURE_DIR "monitor/textures"
#endif

namespace monitor
{

namespace
{

// Push constant block shared by every pipeline; must match the GLSL `PC`.
struct PushConstants
{
  glm::mat4 mvp;
  glm::vec4 color;
};

// Log and return false -- keeps the setup chain terse (if (!step()) return).
bool fail(const char* what)
{
  std::fprintf(stderr, "[monitor] Vulkan setup failed: %s\n", what);
  return false;
}

bool vk_ok(VkResult r)
{
  return r == VK_SUCCESS;
}

}  // namespace

VulkanRenderer::VulkanRenderer(GLFWwindow* window) : window_(window)
{
  // Fail-soft: any step returning false leaves ok_ == false and the caller
  // exits cleanly. The renderer succeeding is explicitly secondary to never
  // disturbing the simulator, so we never throw or abort the process.
  if (!create_instance())
  {
    return;
  }
  if (!create_surface(window))
  {
    return;
  }
  if (!pick_device())
  {
    return;
  }
  if (!create_device())
  {
    return;
  }
  if (!create_swapchain())
  {
    return;
  }
  if (!create_depth())
  {
    return;
  }
  if (!create_commands_and_sync())
  {
    return;
  }
  if (!create_pipelines())
  {
    return;
  }
  ok_ = true;
  init_imgui(window);  // best-effort; the scene still renders if the UI fails
}

VulkanRenderer::VulkanRenderer(int width, int height)
{
  // Headless: no window, no surface, no swapchain -- render straight into an
  // offscreen color image and copy it back. Everything downstream (depth,
  // commands, pipelines) is shared with the windowed path.
  headless_ = true;
  sc_extent_ = {static_cast<std::uint32_t>(width),
                static_cast<std::uint32_t>(height)};
  if (!create_instance())
  {
    return;
  }
  if (!pick_device())
  {
    return;
  }
  if (!create_device())
  {
    return;
  }
  if (!create_offscreen_target())
  {
    return;
  }
  if (!create_depth())
  {
    return;
  }
  if (!create_commands_and_sync())
  {
    return;
  }
  if (!create_pipelines())
  {
    return;
  }
  ok_ = true;
  init_imgui(
      nullptr);  // headless: no GLFW backend, drives display size manually
}

VulkanRenderer::~VulkanRenderer()
{
  if (device_)
  {
    vkDeviceWaitIdle(device_);
  }

  if (imgui_ready_)
  {
    ImGui_ImplVulkan_Shutdown();
    if (!headless_)
    {
      ImGui_ImplGlfw_Shutdown();  // no GLFW backend in headless
    }
    ImGui::DestroyContext();
  }
  if (imgui_pool_)
  {
    vkDestroyDescriptorPool(device_, imgui_pool_, nullptr);
  }

  destroy_buffer(earth_vb_);
  destroy_buffer(earth_ib_);
  destroy_buffer(atmo_vb_);
  destroy_buffer(atmo_ib_);
  destroy_buffer(points_vb_);
  destroy_buffer(edges_vb_);
  destroy_buffer(path_vb_);
  destroy_buffer(sel_vb_);
  destroy_buffer(marker_vb_);
  destroy_buffer(readback_);

  if (pipe_earth_)
  {
    vkDestroyPipeline(device_, pipe_earth_, nullptr);
  }
  if (pipe_atmo_)
  {
    vkDestroyPipeline(device_, pipe_atmo_, nullptr);
  }
  if (pipe_points_)
  {
    vkDestroyPipeline(device_, pipe_points_, nullptr);
  }
  if (pipe_lines_)
  {
    vkDestroyPipeline(device_, pipe_lines_, nullptr);
  }
  if (pipe_route_)
  {
    vkDestroyPipeline(device_, pipe_route_, nullptr);
  }
  if (pipe_sel_mask_)
  {
    vkDestroyPipeline(device_, pipe_sel_mask_, nullptr);
  }
  if (pipe_sel_outline_)
  {
    vkDestroyPipeline(device_, pipe_sel_outline_, nullptr);
  }
  if (pipe_marker_)
  {
    vkDestroyPipeline(device_, pipe_marker_, nullptr);
  }
  if (pipe_layout_)
  {
    vkDestroyPipelineLayout(device_, pipe_layout_, nullptr);
  }
  if (earth_layout_)
  {
    vkDestroyPipelineLayout(device_, earth_layout_, nullptr);
  }

  // Earth texture resources.
  if (tex_view_)
  {
    vkDestroyImageView(device_, tex_view_, nullptr);
  }
  if (tex_image_)
  {
    vkDestroyImage(device_, tex_image_, nullptr);
  }
  if (tex_memory_)
  {
    vkFreeMemory(device_, tex_memory_, nullptr);
  }
  if (tex_sampler_)
  {
    vkDestroySampler(device_, tex_sampler_, nullptr);
  }
  if (tex_pool_)
  {
    vkDestroyDescriptorPool(device_, tex_pool_, nullptr);
  }
  if (tex_set_layout_)
  {
    vkDestroyDescriptorSetLayout(device_, tex_set_layout_, nullptr);
  }

  if (sem_acquire_)
  {
    vkDestroySemaphore(device_, sem_acquire_, nullptr);
  }
  if (sem_render_)
  {
    vkDestroySemaphore(device_, sem_render_, nullptr);
  }
  if (fence_)
  {
    vkDestroyFence(device_, fence_, nullptr);
  }
  if (cmd_pool_)
  {
    vkDestroyCommandPool(device_, cmd_pool_, nullptr);
  }

  destroy_swapchain();
  if (offscreen_image_)
  {
    vkDestroyImage(device_, offscreen_image_, nullptr);
  }
  if (offscreen_memory_)
  {
    vkFreeMemory(device_, offscreen_memory_, nullptr);
  }
  if (device_)
  {
    vkDestroyDevice(device_, nullptr);
  }
  if (surface_)
  {
    vkDestroySurfaceKHR(instance_, surface_, nullptr);
  }
  if (instance_)
  {
    vkDestroyInstance(instance_, nullptr);
  }
}

void VulkanRenderer::wait_idle()
{
  if (device_)
  {
    vkDeviceWaitIdle(device_);
  }
}

// --------------------------------------------------------------------------
// Setup
// --------------------------------------------------------------------------

bool VulkanRenderer::create_instance()
{
  VkApplicationInfo app{};
  app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app.pApplicationName = "leo monitor";
  app.apiVersion = VK_API_VERSION_1_4;

  // Windowed mode needs the surface/platform extensions GLFW reports; headless
  // mode renders offscreen and needs no instance extensions at all.
  std::vector<const char*> exts;
  if (!headless_)
  {
    std::uint32_t glfw_count = 0;
    const char** glfw_ext = glfwGetRequiredInstanceExtensions(&glfw_count);
    if (!glfw_ext)
    {
      return fail("glfwGetRequiredInstanceExtensions (no Vulkan?)");
    }
    exts.assign(glfw_ext, glfw_ext + glfw_count);
  }

  VkInstanceCreateInfo ci{};
  ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  ci.pApplicationInfo = &app;
  ci.enabledExtensionCount = static_cast<std::uint32_t>(exts.size());
  ci.ppEnabledExtensionNames = exts.data();
  if (!vk_ok(vkCreateInstance(&ci, nullptr, &instance_)))
  {
    return fail("vkCreateInstance");
  }
  return true;
}

bool VulkanRenderer::create_surface(GLFWwindow* window)
{
  if (!vk_ok(glfwCreateWindowSurface(instance_, window, nullptr, &surface_)))
  {
    return fail("glfwCreateWindowSurface");
  }
  return true;
}

bool VulkanRenderer::pick_device()
{
  std::uint32_t n = 0;
  vkEnumeratePhysicalDevices(instance_, &n, nullptr);
  if (n == 0)
  {
    return fail("no Vulkan physical devices");
  }
  std::vector<VkPhysicalDevice> devs(n);
  vkEnumeratePhysicalDevices(instance_, &n, devs.data());

  // Pick the first device that has a graphics queue which can also present to
  // our surface. Prefer a discrete GPU if several qualify.
  VkPhysicalDevice best = VK_NULL_HANDLE;
  std::uint32_t best_family = 0;
  bool best_discrete = false;
  for (VkPhysicalDevice d : devs)
  {
    std::uint32_t qn = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
    std::vector<VkQueueFamilyProperties> qs(qn);
    vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qs.data());
    for (std::uint32_t i = 0; i < qn; ++i)
    {
      if (!(qs[i].queueFlags & VK_QUEUE_GRAPHICS_BIT))
      {
        continue;
      }
      // Headless has no surface, so skip the present-support requirement.
      if (!headless_)
      {
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(d, i, surface_, &present);
        if (!present)
        {
          continue;
        }
      }
      VkPhysicalDeviceProperties props;
      vkGetPhysicalDeviceProperties(d, &props);
      const bool discrete =
          props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
      if (best == VK_NULL_HANDLE || (discrete && !best_discrete))
      {
        best = d;
        best_family = i;
        best_discrete = discrete;
      }
      break;
    }
  }
  if (best == VK_NULL_HANDLE)
  {
    return fail("no graphics+present queue family");
  }
  phys_ = best;
  queue_family_ = best_family;
  return true;
}

bool VulkanRenderer::create_device()
{
  const float prio = 1.0f;
  VkDeviceQueueCreateInfo q{};
  q.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  q.queueFamilyIndex = queue_family_;
  q.queueCount = 1;
  q.pQueuePriorities = &prio;

  // Vulkan 1.3+ core features: dynamic rendering and synchronization2. Both are
  // required by this renderer and are guaranteed available on a 1.4 device.
  VkPhysicalDeviceVulkan13Features v13{};
  v13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  v13.dynamicRendering = VK_TRUE;
  v13.synchronization2 = VK_TRUE;

  VkPhysicalDeviceFeatures2 feats{};
  feats.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  feats.pNext = &v13;

  // The swapchain extension is only needed for the windowed present path.
  const char* dev_ext[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};

  VkDeviceCreateInfo ci{};
  ci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  ci.pNext = &feats;  // features2 chain; pEnabledFeatures stays null
  ci.queueCreateInfoCount = 1;
  ci.pQueueCreateInfos = &q;
  ci.enabledExtensionCount = headless_ ? 0u : 1u;
  ci.ppEnabledExtensionNames = headless_ ? nullptr : dev_ext;
  if (!vk_ok(vkCreateDevice(phys_, &ci, nullptr, &device_)))
  {
    return fail("vkCreateDevice");
  }
  vkGetDeviceQueue(device_, queue_family_, 0, &queue_);
  return true;
}

bool VulkanRenderer::create_swapchain()
{
  VkSurfaceCapabilitiesKHR caps;
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys_, surface_, &caps);

  // Choose an sRGB B8G8R8A8 format if available, else whatever is first.
  std::uint32_t fn = 0;
  vkGetPhysicalDeviceSurfaceFormatsKHR(phys_, surface_, &fn, nullptr);
  std::vector<VkSurfaceFormatKHR> formats(fn);
  vkGetPhysicalDeviceSurfaceFormatsKHR(phys_, surface_, &fn, formats.data());
  VkSurfaceFormatKHR chosen = formats[0];
  for (const auto& f : formats)
  {
    if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
        f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
    {
      chosen = f;
      break;
    }
  }
  sc_format_ = chosen.format;

  // Extent: honor currentExtent, else the framebuffer size from GLFW.
  if (caps.currentExtent.width != UINT32_MAX)
  {
    sc_extent_ = caps.currentExtent;
  }
  else
  {
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    sc_extent_.width = static_cast<std::uint32_t>(w);
    sc_extent_.height = static_cast<std::uint32_t>(h);
  }

  std::uint32_t image_count = caps.minImageCount + 1;
  if (caps.maxImageCount > 0 && image_count > caps.maxImageCount)
  {
    image_count = caps.maxImageCount;
  }

  VkSwapchainCreateInfoKHR ci{};
  ci.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  ci.surface = surface_;
  ci.minImageCount = image_count;
  ci.imageFormat = chosen.format;
  ci.imageColorSpace = chosen.colorSpace;
  ci.imageExtent = sc_extent_;
  ci.imageArrayLayers = 1;
  ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
  ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  ci.preTransform = caps.currentTransform;
  ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  ci.presentMode = VK_PRESENT_MODE_FIFO_KHR;  // always supported; vsync
  ci.clipped = VK_TRUE;
  if (!vk_ok(vkCreateSwapchainKHR(device_, &ci, nullptr, &swapchain_)))
  {
    return fail("vkCreateSwapchainKHR");
  }

  vkGetSwapchainImagesKHR(device_, swapchain_, &image_count, nullptr);
  sc_images_.resize(image_count);
  vkGetSwapchainImagesKHR(device_, swapchain_, &image_count, sc_images_.data());

  sc_views_.resize(image_count);
  for (std::uint32_t i = 0; i < image_count; ++i)
  {
    VkImageViewCreateInfo vi{};
    vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vi.image = sc_images_[i];
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = sc_format_;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (!vk_ok(vkCreateImageView(device_, &vi, nullptr, &sc_views_[i])))
    {
      return fail("swapchain image view");
    }
  }
  return true;
}

void VulkanRenderer::destroy_swapchain()
{
  if (depth_view_)
  {
    vkDestroyImageView(device_, depth_view_, nullptr);
    depth_view_ = VK_NULL_HANDLE;
  }
  if (depth_image_)
  {
    vkDestroyImage(device_, depth_image_, nullptr);
    depth_image_ = VK_NULL_HANDLE;
  }
  if (depth_memory_)
  {
    vkFreeMemory(device_, depth_memory_, nullptr);
    depth_memory_ = VK_NULL_HANDLE;
  }
  for (VkImageView v : sc_views_)
  {
    if (v)
    {
      vkDestroyImageView(device_, v, nullptr);
    }
  }
  sc_views_.clear();
  if (swapchain_)
  {
    vkDestroySwapchainKHR(device_, swapchain_, nullptr);
    swapchain_ = VK_NULL_HANDLE;
  }
}

bool VulkanRenderer::recreate_swapchain()
{
  // Spin while minimized (zero-size framebuffer) so we never make a 0x0 chain.
  int w = 0, h = 0;
  glfwGetFramebufferSize(window_, &w, &h);
  while (w == 0 || h == 0)
  {
    glfwWaitEvents();
    glfwGetFramebufferSize(window_, &w, &h);
  }
  vkDeviceWaitIdle(device_);
  destroy_swapchain();
  if (!create_swapchain())
  {
    return false;
  }
  if (!create_depth())
  {
    return false;
  }
  return true;
}

bool VulkanRenderer::create_offscreen_target()
{
  // A single color image we render into and then copy to a buffer. UNORM so the
  // shader's already-display-referred colors are written through unchanged (an
  // sRGB target would gamma-brighten them). TRANSFER_SRC lets us copy it out.
  sc_format_ = VK_FORMAT_R8G8B8A8_UNORM;

  VkImageCreateInfo ic{};
  ic.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  ic.imageType = VK_IMAGE_TYPE_2D;
  ic.format = sc_format_;
  ic.extent = {sc_extent_.width, sc_extent_.height, 1};
  ic.mipLevels = 1;
  ic.arrayLayers = 1;
  ic.samples = VK_SAMPLE_COUNT_1_BIT;
  ic.tiling = VK_IMAGE_TILING_OPTIMAL;
  ic.usage =
      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  ic.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  if (!vk_ok(vkCreateImage(device_, &ic, nullptr, &offscreen_image_)))
  {
    return fail("offscreen image");
  }

  VkMemoryRequirements req;
  vkGetImageMemoryRequirements(device_, offscreen_image_, &req);
  VkMemoryAllocateInfo ai{};
  ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  ai.allocationSize = req.size;
  ai.memoryTypeIndex =
      find_memory_type(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  if (!vk_ok(vkAllocateMemory(device_, &ai, nullptr, &offscreen_memory_)))
  {
    return fail("offscreen memory");
  }
  vkBindImageMemory(device_, offscreen_image_, offscreen_memory_, 0);

  VkImageViewCreateInfo vi{};
  vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  vi.image = offscreen_image_;
  vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
  vi.format = sc_format_;
  vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  VkImageView view = VK_NULL_HANDLE;
  if (!vk_ok(vkCreateImageView(device_, &vi, nullptr, &view)))
  {
    return fail("offscreen view");
  }

  // Present record_scene with a one-entry "swapchain" so it needs no special
  // case: image 0 is our offscreen target.
  sc_images_ = {offscreen_image_};
  sc_views_ = {view};
  return true;
}

bool VulkanRenderer::create_depth()
{
  VkImageCreateInfo ic{};
  ic.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  ic.imageType = VK_IMAGE_TYPE_2D;
  ic.format = depth_format_;
  ic.extent = {sc_extent_.width, sc_extent_.height, 1};
  ic.mipLevels = 1;
  ic.arrayLayers = 1;
  ic.samples = VK_SAMPLE_COUNT_1_BIT;
  ic.tiling = VK_IMAGE_TILING_OPTIMAL;
  ic.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
  ic.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  if (!vk_ok(vkCreateImage(device_, &ic, nullptr, &depth_image_)))
  {
    return fail("depth image");
  }

  VkMemoryRequirements req;
  vkGetImageMemoryRequirements(device_, depth_image_, &req);
  VkMemoryAllocateInfo ai{};
  ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  ai.allocationSize = req.size;
  ai.memoryTypeIndex =
      find_memory_type(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  if (!vk_ok(vkAllocateMemory(device_, &ai, nullptr, &depth_memory_)))
  {
    return fail("depth memory");
  }
  vkBindImageMemory(device_, depth_image_, depth_memory_, 0);

  VkImageViewCreateInfo vi{};
  vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  vi.image = depth_image_;
  vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
  vi.format = depth_format_;
  // Both aspects so the one view serves the depth AND stencil attachments.
  vi.subresourceRange = {
      VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT, 0, 1, 0, 1};
  if (!vk_ok(vkCreateImageView(device_, &vi, nullptr, &depth_view_)))
  {
    return fail("depth view");
  }
  return true;
}

bool VulkanRenderer::create_commands_and_sync()
{
  VkCommandPoolCreateInfo pc{};
  pc.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pc.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pc.queueFamilyIndex = queue_family_;
  if (!vk_ok(vkCreateCommandPool(device_, &pc, nullptr, &cmd_pool_)))
  {
    return fail("command pool");
  }

  VkCommandBufferAllocateInfo ai{};
  ai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  ai.commandPool = cmd_pool_;
  ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  ai.commandBufferCount = 1;
  if (!vk_ok(vkAllocateCommandBuffers(device_, &ai, &cmd_)))
  {
    return fail("command buffer");
  }

  VkSemaphoreCreateInfo si{};
  si.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
  VkFenceCreateInfo fi{};
  fi.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;  // first wait returns immediately
  if (!vk_ok(vkCreateSemaphore(device_, &si, nullptr, &sem_acquire_)) ||
      !vk_ok(vkCreateSemaphore(device_, &si, nullptr, &sem_render_)) ||
      !vk_ok(vkCreateFence(device_, &fi, nullptr, &fence_)))
  {
    return fail("sync objects");
  }
  return true;
}

VkShaderModule VulkanRenderer::load_shader(const char* name)
{
  std::string path = std::string(MONITOR_SHADER_DIR) + "/" + name + ".spv";
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f)
  {
    std::fprintf(stderr, "[monitor] cannot open shader: %s\n", path.c_str());
    return VK_NULL_HANDLE;
  }
  const std::streamsize size = f.tellg();
  std::vector<char> code(static_cast<std::size_t>(size));
  f.seekg(0);
  f.read(code.data(), size);

  VkShaderModuleCreateInfo ci{};
  ci.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  ci.codeSize = code.size();
  ci.pCode = reinterpret_cast<const std::uint32_t*>(code.data());
  VkShaderModule m = VK_NULL_HANDLE;
  vkCreateShaderModule(device_, &ci, nullptr, &m);
  return m;
}

VkPipeline VulkanRenderer::make_pipeline(
    const char* vert, const char* frag, VkPrimitiveTopology topo,
    const VertexInput& vi, bool depth_test, bool depth_write, bool blend,
    VkCullModeFlags cull, VkPipelineLayout layout, bool color_write,
    bool stencil_enable, VkStencilOpState stencil)
{
  VkShaderModule vs = load_shader(vert);
  VkShaderModule fs = load_shader(frag);
  if (!vs || !fs)
  {
    if (vs)
    {
      vkDestroyShaderModule(device_, vs, nullptr);
    }
    if (fs)
    {
      vkDestroyShaderModule(device_, fs, nullptr);
    }
    return VK_NULL_HANDLE;
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vs;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = fs;
  stages[1].pName = "main";

  VkPipelineVertexInputStateCreateInfo vin{};
  vin.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  vin.vertexBindingDescriptionCount =
      static_cast<std::uint32_t>(vi.bindings.size());
  vin.pVertexBindingDescriptions = vi.bindings.data();
  vin.vertexAttributeDescriptionCount =
      static_cast<std::uint32_t>(vi.attrs.size());
  vin.pVertexAttributeDescriptions = vi.attrs.data();

  VkPipelineInputAssemblyStateCreateInfo ia{};
  ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  ia.topology = topo;

  VkPipelineViewportStateCreateInfo vp{};
  vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  vp.viewportCount = 1;
  vp.scissorCount = 1;  // both set dynamically each frame

  VkPipelineRasterizationStateCreateInfo rs{};
  rs.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  rs.polygonMode = VK_POLYGON_MODE_FILL;
  rs.cullMode = cull;
  rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  rs.lineWidth = 1.0f;

  VkPipelineMultisampleStateCreateInfo ms{};
  ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo ds{};
  ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  ds.depthTestEnable = depth_test ? VK_TRUE : VK_FALSE;
  ds.depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE;
  ds.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
  // Selection outline: the mask/outline passes drive the stencil test; the same
  // op state applies to front and back (points have no facing).
  ds.stencilTestEnable = stencil_enable ? VK_TRUE : VK_FALSE;
  ds.front = stencil;
  ds.back = stencil;

  VkPipelineColorBlendAttachmentState cba{};
  // color_write=false (the stencil mask pass) writes nothing to color, only the
  // stencil buffer.
  cba.colorWriteMask =
      color_write ? (VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                     VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT)
                  : 0;
  cba.blendEnable = blend ? VK_TRUE : VK_FALSE;
  cba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
  cba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  cba.colorBlendOp = VK_BLEND_OP_ADD;
  // Keep the DESTINATION alpha (do not overwrite it with the overlay's low
  // alpha). Otherwise a translucent pass leaves the framebuffer nearly
  // transparent, and a screenshot composited over white looks washed out.
  cba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
  cba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  cba.alphaBlendOp = VK_BLEND_OP_ADD;

  VkPipelineColorBlendStateCreateInfo cb{};
  cb.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  cb.attachmentCount = 1;
  cb.pAttachments = &cba;

  const VkDynamicState dyn[] = {VK_DYNAMIC_STATE_VIEWPORT,
                                VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynst{};
  dynst.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynst.dynamicStateCount = 2;
  dynst.pDynamicStates = dyn;

  // Dynamic rendering: declare the attachment formats instead of a render pass.
  VkPipelineRenderingCreateInfo rendering{};
  rendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  rendering.colorAttachmentCount = 1;
  rendering.pColorAttachmentFormats = &sc_format_;
  rendering.depthAttachmentFormat = depth_format_;
  rendering.stencilAttachmentFormat = depth_format_;  // combined D32S8 view

  VkGraphicsPipelineCreateInfo pi{};
  pi.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pi.pNext = &rendering;
  pi.stageCount = 2;
  pi.pStages = stages;
  pi.pVertexInputState = &vin;
  pi.pInputAssemblyState = &ia;
  pi.pViewportState = &vp;
  pi.pRasterizationState = &rs;
  pi.pMultisampleState = &ms;
  pi.pDepthStencilState = &ds;
  pi.pColorBlendState = &cb;
  pi.pDynamicState = &dynst;
  pi.layout = layout;

  VkPipeline pipe = VK_NULL_HANDLE;
  vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &pi, nullptr, &pipe);
  vkDestroyShaderModule(device_, vs, nullptr);
  vkDestroyShaderModule(device_, fs, nullptr);
  return pipe;
}

bool VulkanRenderer::create_pipelines()
{
  VkPushConstantRange pcr{};
  pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  pcr.offset = 0;
  pcr.size = sizeof(PushConstants);

  VkPipelineLayoutCreateInfo li{};
  li.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  li.pushConstantRangeCount = 1;
  li.pPushConstantRanges = &pcr;
  if (!vk_ok(vkCreatePipelineLayout(device_, &li, nullptr, &pipe_layout_)))
  {
    return fail("pipeline layout");
  }

  // Texture descriptor set layout + sampler/pool/set, then a second pipeline
  // layout that also binds the Earth texture at set 0.
  if (!create_texture_infra())
  {
    return false;
  }
  VkPipelineLayoutCreateInfo eli{};
  eli.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  eli.setLayoutCount = 1;
  eli.pSetLayouts = &tex_set_layout_;
  eli.pushConstantRangeCount = 1;
  eli.pPushConstantRanges = &pcr;
  if (!vk_ok(vkCreatePipelineLayout(device_, &eli, nullptr, &earth_layout_)))
  {
    return fail("earth pipeline layout");
  }

  // Vertex layouts. MeshVertex = (pos, normal); LineVertex = (pos, color);
  // points are a bare vec3.
  VertexInput mesh_vi;
  mesh_vi.bindings = {{0, sizeof(MeshVertex), VK_VERTEX_INPUT_RATE_VERTEX}};
  mesh_vi.attrs = {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, pos)},
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(MeshVertex, normal)}};

  VertexInput line_vi;
  line_vi.bindings = {{0, sizeof(LineVertex), VK_VERTEX_INPUT_RATE_VERTEX}};
  line_vi.attrs = {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LineVertex, pos)},
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(LineVertex, color)}};

  VertexInput point_vi;
  point_vi.bindings = {{0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX}};
  point_vi.attrs = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}};

  // RouteVertex = (pos, other, color, side, across): the billboard route quads.
  VertexInput route_vi;
  route_vi.bindings = {{0, sizeof(RouteVertex), VK_VERTEX_INPUT_RATE_VERTEX}};
  route_vi.attrs = {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(RouteVertex, pos)},
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(RouteVertex, other)},
      {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(RouteVertex, color)},
      {3, 0, VK_FORMAT_R32_SFLOAT, offsetof(RouteVertex, side)},
      {4, 0, VK_FORMAT_R32_SFLOAT, offsetof(RouteVertex, across)}};

  // Earth: opaque, depth test+write, cull back (near hemisphere is front-facing
  // after the build_sphere winding fix). Textured -> uses earth_layout_.
  pipe_earth_ = make_pipeline(
      "earth.vert", "earth.frag", VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, mesh_vi,
      true, true, false, VK_CULL_MODE_BACK_BIT, earth_layout_);
  // Atmosphere: translucent, depth test but no write, drawn last.
  pipe_atmo_ = make_pipeline("mesh.vert", "mesh.frag",
                             VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, mesh_vi, true,
                             false, true, VK_CULL_MODE_BACK_BIT, pipe_layout_);
  // Satellites: point cloud.
  pipe_points_ = make_pipeline("point.vert", "point.frag",
                               VK_PRIMITIVE_TOPOLOGY_POINT_LIST, point_vi, true,
                               true, false, VK_CULL_MODE_NONE, pipe_layout_);
  // Edges: colored 1px lines (the link mesh).
  pipe_lines_ = make_pipeline("line.vert", "line.frag",
                              VK_PRIMITIVE_TOPOLOGY_LINE_LIST, line_vi, true,
                              false, false, VK_CULL_MODE_NONE, pipe_layout_);
  // Route: thick billboard triangles, ALWAYS ON TOP. depth_test = false (option
  // (a) from the spec) so the highlight is never occluded by satellites or the
  // mesh -- it is an overlay, not physical geometry, and must be followable end
  // to end from any angle. Blend on for the soft-edged core; cull nothing since
  // the expanded quads have arbitrary winding.
  pipe_route_ = make_pipeline(
      "route.vert", "route.frag", VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, route_vi,
      false, false, true, VK_CULL_MODE_NONE, pipe_layout_);

  // Selection highlight = a crisp stencil ring around the picked dot, drawn as
  // two point passes (size in the push-constant alpha). Stencil is used instead
  // of a fixed marker so the ring width is independent of the dot's own size:
  //   mask pass    -- an inner disc writes stencil=1, no color.
  //   outline pass -- a larger disc draws colour ONLY where stencil != 1, i.e.
  //                   the annulus outside the inner disc -> a clean ring.
  // Both skip the depth test so the ring always tracks and shows the selection.
  VkStencilOpState sel_mask{};
  sel_mask.failOp = VK_STENCIL_OP_KEEP;
  sel_mask.passOp = VK_STENCIL_OP_REPLACE;  // mark the inner disc
  sel_mask.depthFailOp = VK_STENCIL_OP_KEEP;
  sel_mask.compareOp = VK_COMPARE_OP_ALWAYS;
  sel_mask.compareMask = 0xFF;
  sel_mask.writeMask = 0xFF;
  sel_mask.reference = 1;
  pipe_sel_mask_ = make_pipeline("select.vert", "select.frag",
                                 VK_PRIMITIVE_TOPOLOGY_POINT_LIST, point_vi,
                                 false, false, false, VK_CULL_MODE_NONE,
                                 pipe_layout_, false, true, sel_mask);

  VkStencilOpState sel_ring{};
  sel_ring.failOp = VK_STENCIL_OP_KEEP;
  sel_ring.passOp = VK_STENCIL_OP_KEEP;
  sel_ring.depthFailOp = VK_STENCIL_OP_KEEP;
  sel_ring.compareOp = VK_COMPARE_OP_NOT_EQUAL;  // draw only outside the disc
  sel_ring.compareMask = 0xFF;
  sel_ring.writeMask = 0x00;  // the ring pass never changes stencil
  sel_ring.reference = 1;
  pipe_sel_outline_ = make_pipeline("select.vert", "select.frag",
                                    VK_PRIMITIVE_TOPOLOGY_POINT_LIST, point_vi,
                                    false, false, false, VK_CULL_MODE_NONE,
                                    pipe_layout_, true, true, sel_ring);

  // Blind-endpoint marker: a plain amber ring (marker.frag draws the annulus),
  // no stencil needed. Depth test off so the marker is always visible at the
  // flagged ground point. Reuses select.vert (point size/color via push const).
  pipe_marker_ = make_pipeline(
      "select.vert", "marker.frag", VK_PRIMITIVE_TOPOLOGY_POINT_LIST, point_vi,
      false, false, false, VK_CULL_MODE_NONE, pipe_layout_);

  if (!pipe_earth_ || !pipe_atmo_ || !pipe_points_ || !pipe_lines_ ||
      !pipe_route_ || !pipe_sel_mask_ || !pipe_sel_outline_ || !pipe_marker_)
  {
    return fail("graphics pipelines (missing .spv shaders?)");
  }

  // Load the day-map (falls back to a 1x1 white texture if the file is absent).
  load_earth_texture(std::string(MONITOR_TEXTURE_DIR) + "/2k_earth_daymap.jpg");
  return true;
}

bool VulkanRenderer::create_texture_infra()
{
  VkDescriptorSetLayoutBinding b{};
  b.binding = 0;
  b.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  b.descriptorCount = 1;
  b.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
  VkDescriptorSetLayoutCreateInfo li{};
  li.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  li.bindingCount = 1;
  li.pBindings = &b;
  if (!vk_ok(
          vkCreateDescriptorSetLayout(device_, &li, nullptr, &tex_set_layout_)))
  {
    return fail("texture set layout");
  }

  VkSamplerCreateInfo si{};
  si.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  si.magFilter = VK_FILTER_LINEAR;
  si.minFilter = VK_FILTER_LINEAR;
  si.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;         // longitude wraps
  si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;  // latitude clamps
  si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  si.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  si.maxLod = VK_LOD_CLAMP_NONE;
  if (!vk_ok(vkCreateSampler(device_, &si, nullptr, &tex_sampler_)))
  {
    return fail("sampler");
  }

  VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
  VkDescriptorPoolCreateInfo pi{};
  pi.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pi.maxSets = 1;
  pi.poolSizeCount = 1;
  pi.pPoolSizes = &ps;
  if (!vk_ok(vkCreateDescriptorPool(device_, &pi, nullptr, &tex_pool_)))
  {
    return fail("texture pool");
  }

  VkDescriptorSetAllocateInfo ai{};
  ai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  ai.descriptorPool = tex_pool_;
  ai.descriptorSetCount = 1;
  ai.pSetLayouts = &tex_set_layout_;
  if (!vk_ok(vkAllocateDescriptorSets(device_, &ai, &tex_set_)))
  {
    return fail("texture descriptor set");
  }
  return true;
}

bool VulkanRenderer::load_earth_texture(const std::string& path)
{
  int w = 0, h = 0, n = 0;
  unsigned char* pixels = stbi_load(path.c_str(), &w, &h, &n, 4);  // force RGBA
  if (!pixels)
  {
    std::fprintf(stderr,
                 "[monitor] earth texture '%s' not loaded (%s); using plain "
                 "sphere\n",
                 path.c_str(), stbi_failure_reason());
    const unsigned char white[4] = {255, 255, 255, 255};  // 1x1 fallback
    return upload_texture_rgba(white, 1, 1);
  }
  const bool ok = upload_texture_rgba(pixels, w, h);
  stbi_image_free(pixels);
  if (ok)
  {
    std::printf("[monitor] earth texture: %dx%d from %s\n", w, h, path.c_str());
  }
  return ok;
}

bool VulkanRenderer::upload_texture_rgba(const unsigned char* rgba, int w,
                                         int h)
{
  const VkDeviceSize bytes = static_cast<VkDeviceSize>(w) * h * 4;

  // Replace any existing texture (re-upload): the GPU is idle at load time.
  if (tex_view_)
  {
    vkDeviceWaitIdle(device_);
    vkDestroyImageView(device_, tex_view_, nullptr);
    vkDestroyImage(device_, tex_image_, nullptr);
    vkFreeMemory(device_, tex_memory_, nullptr);
    tex_view_ = VK_NULL_HANDLE;
    tex_image_ = VK_NULL_HANDLE;
    tex_memory_ = VK_NULL_HANDLE;
  }

  // Host-visible staging buffer with the pixels.
  Buffer staging;
  VkBufferCreateInfo bci{};
  bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bci.size = bytes;
  bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  vkCreateBuffer(device_, &bci, nullptr, &staging.buffer);
  VkMemoryRequirements sreq;
  vkGetBufferMemoryRequirements(device_, staging.buffer, &sreq);
  VkMemoryAllocateInfo sai{};
  sai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  sai.allocationSize = sreq.size;
  sai.memoryTypeIndex = find_memory_type(
      sreq.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                               VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
  vkAllocateMemory(device_, &sai, nullptr, &staging.memory);
  vkBindBufferMemory(device_, staging.buffer, staging.memory, 0);
  vkMapMemory(device_, staging.memory, 0, VK_WHOLE_SIZE, 0, &staging.mapped);
  std::memcpy(staging.mapped, rgba, static_cast<std::size_t>(bytes));

  // Device-local sampled image.
  VkImageCreateInfo ic{};
  ic.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  ic.imageType = VK_IMAGE_TYPE_2D;
  ic.format = VK_FORMAT_R8G8B8A8_UNORM;
  ic.extent = {static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h), 1};
  ic.mipLevels = 1;
  ic.arrayLayers = 1;
  ic.samples = VK_SAMPLE_COUNT_1_BIT;
  ic.tiling = VK_IMAGE_TILING_OPTIMAL;
  ic.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
  ic.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  vkCreateImage(device_, &ic, nullptr, &tex_image_);
  VkMemoryRequirements ireq;
  vkGetImageMemoryRequirements(device_, tex_image_, &ireq);
  VkMemoryAllocateInfo iai{};
  iai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  iai.allocationSize = ireq.size;
  iai.memoryTypeIndex = find_memory_type(ireq.memoryTypeBits,
                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
  vkAllocateMemory(device_, &iai, nullptr, &tex_memory_);
  vkBindImageMemory(device_, tex_image_, tex_memory_, 0);

  // One-time upload: UNDEFINED -> TRANSFER_DST, copy, -> SHADER_READ_ONLY.
  vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);
  vkResetFences(device_, 1, &fence_);
  vkResetCommandBuffer(cmd_, 0);
  VkCommandBufferBeginInfo cbi{};
  cbi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  cbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cmd_, &cbi);

  const auto barrier = [&](VkImageLayout from, VkImageLayout to,
                           VkPipelineStageFlags2 ss, VkAccessFlags2 sa,
                           VkPipelineStageFlags2 ds, VkAccessFlags2 da)
  {
    VkImageMemoryBarrier2 b{};
    b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    b.srcStageMask = ss;
    b.srcAccessMask = sa;
    b.dstStageMask = ds;
    b.dstAccessMask = da;
    b.oldLayout = from;
    b.newLayout = to;
    b.image = tex_image_;
    b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    VkDependencyInfo dep{};
    dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dep.imageMemoryBarrierCount = 1;
    dep.pImageMemoryBarriers = &b;
    vkCmdPipelineBarrier2(cmd_, &dep);
  };

  barrier(VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
          VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0, VK_PIPELINE_STAGE_2_COPY_BIT,
          VK_ACCESS_2_TRANSFER_WRITE_BIT);
  VkBufferImageCopy region{};
  region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  region.imageExtent = {static_cast<std::uint32_t>(w),
                        static_cast<std::uint32_t>(h), 1};
  vkCmdCopyBufferToImage(cmd_, staging.buffer, tex_image_,
                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
  barrier(VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
          VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
          VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
          VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
  vkEndCommandBuffer(cmd_);

  VkCommandBufferSubmitInfo csi{};
  csi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
  csi.commandBuffer = cmd_;
  VkSubmitInfo2 submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
  submit.commandBufferInfoCount = 1;
  submit.pCommandBufferInfos = &csi;
  vkQueueSubmit2(queue_, 1, &submit, fence_);
  vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);
  destroy_buffer(staging);

  // View + point the descriptor set at the new texture.
  VkImageViewCreateInfo vi{};
  vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  vi.image = tex_image_;
  vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
  vi.format = VK_FORMAT_R8G8B8A8_UNORM;
  vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  vkCreateImageView(device_, &vi, nullptr, &tex_view_);

  VkDescriptorImageInfo dii{};
  dii.sampler = tex_sampler_;
  dii.imageView = tex_view_;
  dii.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  VkWriteDescriptorSet wr{};
  wr.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  wr.dstSet = tex_set_;
  wr.dstBinding = 0;
  wr.descriptorCount = 1;
  wr.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  wr.pImageInfo = &dii;
  vkUpdateDescriptorSets(device_, 1, &wr, 0, nullptr);
  return true;
}

void VulkanRenderer::init_imgui(GLFWwindow* window)
{
  // A small descriptor pool for ImGui's font atlas (and any user textures).
  VkDescriptorPoolSize sizes[] = {
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 8}};
  VkDescriptorPoolCreateInfo pci{};
  pci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pci.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
  pci.maxSets = 8;
  pci.poolSizeCount = 1;
  pci.pPoolSizes = sizes;
  if (!vk_ok(vkCreateDescriptorPool(device_, &pci, nullptr, &imgui_pool_)))
  {
    std::fprintf(stderr, "[monitor] ImGui pool failed; UI disabled\n");
    return;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  // Headless (window == nullptr) skips the GLFW backend and drives the display
  // size manually -- lets a screenshot render the UI with no visible window.
  if (window)
  {
    ImGui_ImplGlfw_InitForVulkan(window, /*install_callbacks=*/true);
  }

  // Dynamic rendering: no VkRenderPass, just the color format ImGui draws into.
  ImGui_ImplVulkan_InitInfo info{};
  info.Instance = instance_;
  info.PhysicalDevice = phys_;
  info.Device = device_;
  info.QueueFamily = queue_family_;
  info.Queue = queue_;
  info.DescriptorPool = imgui_pool_;
  // ImGui requires >= 2 (double-buffering assumption); the headless path has a
  // single offscreen image, so clamp -- it only sizes ImGui's internal buffers.
  const std::uint32_t imgs =
      std::max<std::uint32_t>(2, static_cast<std::uint32_t>(sc_images_.size()));
  info.MinImageCount = imgs;
  info.ImageCount = imgs;
  info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
  info.UseDynamicRendering = true;
  // ImGui draws into a color-only attachment of the swapchain format. The
  // pointer targets the stable sc_format_ member (valid for the renderer's
  // life).
  info.PipelineRenderingCreateInfo = {};
  info.PipelineRenderingCreateInfo.sType =
      VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
  info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &sc_format_;
  ImGui_ImplVulkan_Init(&info);  // font atlas is created lazily on first frame

  imgui_ready_ = true;
}

// --------------------------------------------------------------------------
// Buffers
// --------------------------------------------------------------------------

std::uint32_t VulkanRenderer::find_memory_type(
    std::uint32_t type_bits, VkMemoryPropertyFlags props) const
{
  VkPhysicalDeviceMemoryProperties mp;
  vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
  for (std::uint32_t i = 0; i < mp.memoryTypeCount; ++i)
  {
    if ((type_bits & (1u << i)) &&
        (mp.memoryTypes[i].propertyFlags & props) == props)
    {
      return i;
    }
  }
  return 0;  // best effort; allocation will simply fail downstream if wrong
}

void VulkanRenderer::destroy_buffer(Buffer& b)
{
  if (b.mapped)
  {
    vkUnmapMemory(device_, b.memory);
    b.mapped = nullptr;
  }
  if (b.buffer)
  {
    vkDestroyBuffer(device_, b.buffer, nullptr);
  }
  if (b.memory)
  {
    vkFreeMemory(device_, b.memory, nullptr);
  }
  b.buffer = VK_NULL_HANDLE;
  b.memory = VK_NULL_HANDLE;
  b.capacity = 0;
  b.count = 0;
}

void VulkanRenderer::upload(Buffer& b, const void* data, VkDeviceSize bytes,
                            VkBufferUsageFlags usage, std::uint32_t count)
{
  b.count = count;
  if (bytes == 0)
  {
    return;
  }
  // Grow (never shrink) so steady-state ticks reuse the same allocation.
  if (bytes > b.capacity)
  {
    if (b.buffer)
    {
      vkDeviceWaitIdle(device_);  // not in flight: safe to replace
      destroy_buffer(b);
    }
    VkBufferCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    ci.size = bytes;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    vkCreateBuffer(device_, &ci, nullptr, &b.buffer);

    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device_, b.buffer, &req);
    VkMemoryAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = find_memory_type(
        req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    vkAllocateMemory(device_, &ai, nullptr, &b.memory);
    vkBindBufferMemory(device_, b.buffer, b.memory, 0);
    vkMapMemory(device_, b.memory, 0, VK_WHOLE_SIZE, 0, &b.mapped);
    b.capacity = req.size;
  }
  std::memcpy(b.mapped, data, static_cast<std::size_t>(bytes));
}

void VulkanRenderer::set_earth_mesh(const std::vector<MeshVertex>& v,
                                    const std::vector<std::uint32_t>& idx)
{
  upload(earth_vb_, v.data(), v.size() * sizeof(MeshVertex),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(v.size()));
  upload(earth_ib_, idx.data(), idx.size() * sizeof(std::uint32_t),
         VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
         static_cast<std::uint32_t>(idx.size()));
}

void VulkanRenderer::set_atmosphere_mesh(const std::vector<MeshVertex>& v,
                                         const std::vector<std::uint32_t>& idx)
{
  upload(atmo_vb_, v.data(), v.size() * sizeof(MeshVertex),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(v.size()));
  upload(atmo_ib_, idx.data(), idx.size() * sizeof(std::uint32_t),
         VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
         static_cast<std::uint32_t>(idx.size()));
}

void VulkanRenderer::set_points(const std::vector<glm::vec3>& pts)
{
  upload(points_vb_, pts.data(), pts.size() * sizeof(glm::vec3),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(pts.size()));
}

void VulkanRenderer::set_edges(const std::vector<LineVertex>& lines)
{
  upload(edges_vb_, lines.data(), lines.size() * sizeof(LineVertex),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(lines.size()));
}

void VulkanRenderer::set_path(const std::vector<RouteVertex>& verts)
{
  upload(path_vb_, verts.data(), verts.size() * sizeof(RouteVertex),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(verts.size()));
}

void VulkanRenderer::set_selection(const std::vector<glm::vec3>& pt)
{
  // 0 or 1 world-space point; count 0 disables the highlight draw in record.
  upload(sel_vb_, pt.data(), pt.size() * sizeof(glm::vec3),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(pt.size()));
}

void VulkanRenderer::set_markers(const std::vector<glm::vec3>& pts)
{
  // 0..2 blind-endpoint points; count 0 disables the marker draw in record.
  upload(marker_vb_, pts.data(), pts.size() * sizeof(glm::vec3),
         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
         static_cast<std::uint32_t>(pts.size()));
}

// --------------------------------------------------------------------------
// Frame
// --------------------------------------------------------------------------

namespace
{

// A synchronization2 image layout transition baked into a barrier struct.
VkImageMemoryBarrier2 image_barrier(VkImage image, VkImageLayout from,
                                    VkImageLayout to,
                                    VkPipelineStageFlags2 src_stage,
                                    VkAccessFlags2 src_access,
                                    VkPipelineStageFlags2 dst_stage,
                                    VkAccessFlags2 dst_access,
                                    VkImageAspectFlags aspect)
{
  VkImageMemoryBarrier2 b{};
  b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  b.srcStageMask = src_stage;
  b.srcAccessMask = src_access;
  b.dstStageMask = dst_stage;
  b.dstAccessMask = dst_access;
  b.oldLayout = from;
  b.newLayout = to;
  b.image = image;
  b.subresourceRange = {aspect, 0, 1, 0, 1};
  return b;
}

void pipeline_barrier(VkCommandBuffer cmd, const VkImageMemoryBarrier2& b)
{
  VkDependencyInfo dep{};
  dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dep.imageMemoryBarrierCount = 1;
  dep.pImageMemoryBarriers = &b;
  vkCmdPipelineBarrier2(cmd, &dep);
}

}  // namespace

void VulkanRenderer::record_scene(std::uint32_t image_index,
                                  const glm::mat4& view_proj)
{
  // Caller has already transitioned the color (image_index) and depth images to
  // their attachment layouts. This is the shared draw used by both the windowed
  // present path and the headless capture path.
  VkRenderingAttachmentInfo color{};
  color.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  color.imageView = sc_views_[image_index];
  color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  color.clearValue.color = {{0.02f, 0.03f, 0.06f, 1.0f}};

  VkRenderingAttachmentInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  depth.imageView = depth_view_;
  depth.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
  depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depth.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depth.clearValue.depthStencil = {1.0f, 0};

  // Stencil shares the depth image/view; cleared to 0 so the selection mask
  // pass starts from a blank stencil each frame.
  VkRenderingAttachmentInfo stencil = depth;
  stencil.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  stencil.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;

  VkRenderingInfo ri{};
  ri.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  ri.renderArea.extent = sc_extent_;
  ri.layerCount = 1;
  ri.colorAttachmentCount = 1;
  ri.pColorAttachments = &color;
  ri.pDepthAttachment = &depth;
  ri.pStencilAttachment = &stencil;
  vkCmdBeginRendering(cmd_, &ri);

  VkViewport vp{};
  vp.width = static_cast<float>(sc_extent_.width);
  vp.height = static_cast<float>(sc_extent_.height);
  vp.minDepth = 0.0f;
  vp.maxDepth = 1.0f;
  vkCmdSetViewport(cmd_, 0, 1, &vp);
  VkRect2D sc{};
  sc.extent = sc_extent_;
  vkCmdSetScissor(cmd_, 0, 1, &sc);

  PushConstants pc{};
  pc.mvp = view_proj;
  const VkDeviceSize zero = 0;
  const VkShaderStageFlags stages =
      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

  // Earth (opaque, textured). Uses earth_layout_ and binds the day-map at set
  // 0.
  if (earth_ib_.count)
  {
    pc.color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);  // unused by earth.frag
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_earth_);
    vkCmdBindDescriptorSets(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            earth_layout_, 0, 1, &tex_set_, 0, nullptr);
    vkCmdPushConstants(cmd_, earth_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindVertexBuffers(cmd_, 0, 1, &earth_vb_.buffer, &zero);
    vkCmdBindIndexBuffer(cmd_, earth_ib_.buffer, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmd_, earth_ib_.count, 1, 0, 0, 0);
  }

  // Satellites (point cloud).
  if (points_vb_.count)
  {
    pc.color = glm::vec4(0.90f, 0.92f, 0.96f, 1.0f);
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_points_);
    vkCmdBindVertexBuffers(cmd_, 0, 1, &points_vb_.buffer, &zero);
    vkCmdDraw(cmd_, points_vb_.count, 1, 0, 0);
  }

  // Lattice edges (colored lines).
  if (edges_vb_.count)
  {
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_lines_);
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindVertexBuffers(cmd_, 0, 1, &edges_vb_.buffer, &zero);
    vkCmdDraw(cmd_, edges_vb_.count, 1, 0, 0);
  }

  // Atmosphere (translucent haze over the globe). Alpha is low: with the sphere
  // winding fixed its near hemisphere now draws over the whole globe, so a
  // lighter haze keeps the Earth readable instead of washing it out.
  if (atmo_ib_.count)
  {
    pc.color = glm::vec4(0.40f, 0.60f, 1.0f, 0.15f);
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_atmo_);
    vkCmdBindVertexBuffers(cmd_, 0, 1, &atmo_vb_.buffer, &zero);
    vkCmdBindIndexBuffer(cmd_, atmo_ib_.buffer, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmd_, atmo_ib_.count, 1, 0, 0, 0);
  }

  // Route highlight -- drawn LAST, after every other element (Earth,
  // satellites, link mesh, atmosphere), on the always-on-top pipe_route_ (depth
  // test off) so it is the most prominent thing on screen from any angle. The
  // route shader repurposes pc.color as (viewport_w, viewport_h, half_width_px)
  // to expand the segment quads to a fixed pixel width; 3px half-width => a
  // ~6px band, clearly thicker than the 1px link lines.
  if (path_vb_.count)
  {
    constexpr float kRouteHalfWidthPx = 3.0f;
    pc.color = glm::vec4(static_cast<float>(sc_extent_.width),
                         static_cast<float>(sc_extent_.height),
                         kRouteHalfWidthPx, 0.0f);
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_route_);
    vkCmdBindVertexBuffers(cmd_, 0, 1, &path_vb_.buffer, &zero);
    vkCmdDraw(cmd_, path_vb_.count, 1, 0, 0);
  }

  // Selection highlight (last, on top). Two passes over the single selected
  // point: the mask pass stamps an inner disc into the stencil, the outline
  // pass draws a larger disc only where stencil is clear -> a ring. pc.color.a
  // carries the point size (px) read by select.vert; pc.color.rgb is the ring
  // colour.
  if (sel_vb_.count)
  {
    constexpr float kSelInnerPx = 16.0f;  // inner disc (masked out)
    constexpr float kSelOuterPx = 24.0f;  // outer disc -> ring width ~4 px
    vkCmdBindVertexBuffers(cmd_, 0, 1, &sel_vb_.buffer, &zero);

    pc.color =
        glm::vec4(0.0f, 0.0f, 0.0f, kSelInnerPx);  // rgb unused (no color)
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_sel_mask_);
    vkCmdDraw(cmd_, sel_vb_.count, 1, 0, 0);

    pc.color = glm::vec4(1.0f, 0.15f, 0.85f, kSelOuterPx);  // magenta ring
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_sel_outline_);
    vkCmdDraw(cmd_, sel_vb_.count, 1, 0, 0);
  }

  // Blind-endpoint markers: amber rings at ground points with no coverage.
  if (marker_vb_.count)
  {
    constexpr float kMarkerPx = 26.0f;  // larger than the selection ring
    pc.color = glm::vec4(1.0f, 0.6f, 0.1f, kMarkerPx);  // amber
    vkCmdPushConstants(cmd_, pipe_layout_, stages, 0, sizeof(pc), &pc);
    vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_marker_);
    vkCmdBindVertexBuffers(cmd_, 0, 1, &marker_vb_.buffer, &zero);
    vkCmdDraw(cmd_, marker_vb_.count, 1, 0, 0);
  }

  vkCmdEndRendering(cmd_);
}

void VulkanRenderer::record(std::uint32_t image_index,
                            const glm::mat4& view_proj)
{
  VkCommandBufferBeginInfo bi{};
  bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cmd_, &bi);

  // Swapchain image: UNDEFINED -> COLOR_ATTACHMENT_OPTIMAL for rendering.
  pipeline_barrier(
      cmd_, image_barrier(sc_images_[image_index], VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          VK_IMAGE_ASPECT_COLOR_BIT));
  // Depth+stencil: UNDEFINED -> attachment layout (cleared each frame anyway).
  pipeline_barrier(
      cmd_,
      image_barrier(depth_image_, VK_IMAGE_LAYOUT_UNDEFINED,
                    VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                    VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                        VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                    VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                    VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT));

  record_scene(image_index, view_proj);

  // ImGui overlay: a second dynamic-rendering pass into the SAME color image,
  // loading (not clearing) the scene and with no depth. ImGui's pipeline was
  // created for a color-only attachment, so it cannot share the scene pass
  // (which has a depth attachment). A same-layout barrier orders the scene's
  // color writes before ImGui's load.
  if (imgui_ready_)
  {
    pipeline_barrier(
        cmd_, image_barrier(sc_images_[image_index],
                            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT |
                                VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT,
                            VK_IMAGE_ASPECT_COLOR_BIT));

    VkRenderingAttachmentInfo ui_color{};
    ui_color.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    ui_color.imageView = sc_views_[image_index];
    ui_color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    ui_color.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    ui_color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

    VkRenderingInfo ui_ri{};
    ui_ri.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    ui_ri.renderArea.extent = sc_extent_;
    ui_ri.layerCount = 1;
    ui_ri.colorAttachmentCount = 1;
    ui_ri.pColorAttachments = &ui_color;
    vkCmdBeginRendering(cmd_, &ui_ri);
    if (ImDrawData* dd = ImGui::GetDrawData())
    {
      ImGui_ImplVulkan_RenderDrawData(dd, cmd_);
    }
    vkCmdEndRendering(cmd_);
  }

  // COLOR_ATTACHMENT_OPTIMAL -> PRESENT_SRC for the presentation engine.
  pipeline_barrier(
      cmd_, image_barrier(sc_images_[image_index],
                          VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, 0,
                          VK_IMAGE_ASPECT_COLOR_BIT));
  vkEndCommandBuffer(cmd_);
}

void VulkanRenderer::draw(const glm::mat4& view, const glm::mat4& proj)
{
  if (!ok_)
  {
    return;
  }

  // One frame in flight: wait for the previous submit to fully retire so the
  // single command buffer and semaphores are free to reuse.
  vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);

  std::uint32_t image_index = 0;
  VkResult acq =
      vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX, sem_acquire_,
                            VK_NULL_HANDLE, &image_index);
  if (acq == VK_ERROR_OUT_OF_DATE_KHR)
  {
    recreate_swapchain();
    return;
  }

  vkResetFences(device_, 1, &fence_);
  vkResetCommandBuffer(cmd_, 0);
  record(image_index, proj * view);

  // sync2 submit: wait on the acquire semaphore at color-output, signal the
  // render-finished semaphore for present, fence the whole submit.
  VkCommandBufferSubmitInfo cmd_si{};
  cmd_si.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
  cmd_si.commandBuffer = cmd_;

  VkSemaphoreSubmitInfo wait_si{};
  wait_si.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
  wait_si.semaphore = sem_acquire_;
  wait_si.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

  VkSemaphoreSubmitInfo signal_si{};
  signal_si.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
  signal_si.semaphore = sem_render_;
  signal_si.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

  VkSubmitInfo2 submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
  submit.waitSemaphoreInfoCount = 1;
  submit.pWaitSemaphoreInfos = &wait_si;
  submit.commandBufferInfoCount = 1;
  submit.pCommandBufferInfos = &cmd_si;
  submit.signalSemaphoreInfoCount = 1;
  submit.pSignalSemaphoreInfos = &signal_si;
  vkQueueSubmit2(queue_, 1, &submit, fence_);

  VkPresentInfoKHR present{};
  present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  present.waitSemaphoreCount = 1;
  present.pWaitSemaphores = &sem_render_;
  present.swapchainCount = 1;
  present.pSwapchains = &swapchain_;
  present.pImageIndices = &image_index;
  VkResult pr = vkQueuePresentKHR(queue_, &present);
  if (pr == VK_ERROR_OUT_OF_DATE_KHR || pr == VK_SUBOPTIMAL_KHR)
  {
    recreate_swapchain();
  }
}

// --------------------------------------------------------------------------
// Headless capture: write a PNG with a tiny self-contained encoder (no deps).
// The DEFLATE stream uses STORED (uncompressed) blocks, which is valid zlib;
// the file is a touch larger but the encoder is small and obviously correct.
// --------------------------------------------------------------------------

namespace
{

std::uint32_t crc32_of(const unsigned char* data, std::size_t n)
{
  static std::uint32_t table[256];
  static bool init = false;
  if (!init)
  {
    for (std::uint32_t i = 0; i < 256; ++i)
    {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k)
      {
        c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      }
      table[i] = c;
    }
    init = true;
  }
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < n; ++i)
  {
    crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFFu;
}

void put_be32(std::vector<unsigned char>& v, std::uint32_t x)
{
  v.push_back(static_cast<unsigned char>((x >> 24) & 0xFF));
  v.push_back(static_cast<unsigned char>((x >> 16) & 0xFF));
  v.push_back(static_cast<unsigned char>((x >> 8) & 0xFF));
  v.push_back(static_cast<unsigned char>(x & 0xFF));
}

void put_chunk(std::vector<unsigned char>& out, const char* type,
               const std::vector<unsigned char>& data)
{
  put_be32(out, static_cast<std::uint32_t>(data.size()));
  std::vector<unsigned char> tc(type, type + 4);
  tc.insert(tc.end(), data.begin(), data.end());
  out.insert(out.end(), tc.begin(), tc.end());
  put_be32(out, crc32_of(tc.data(), tc.size()));
}

bool write_png(const char* path, std::uint32_t w, std::uint32_t h,
               const unsigned char* rgba)
{
  // PNG scanlines: a filter byte (0 = none) followed by the RGBA row.
  std::vector<unsigned char> raw;
  raw.reserve(static_cast<std::size_t>(h) *
              (1 + static_cast<std::size_t>(w) * 4));
  for (std::uint32_t y = 0; y < h; ++y)
  {
    raw.push_back(0);
    const unsigned char* row = rgba + static_cast<std::size_t>(y) * w * 4;
    raw.insert(raw.end(), row, row + static_cast<std::size_t>(w) * 4);
  }

  // zlib stream: 2-byte header, then STORED deflate blocks, then Adler-32.
  std::vector<unsigned char> zlib = {0x78, 0x01};
  std::size_t pos = 0;
  while (pos < raw.size() || raw.empty())
  {
    const std::size_t blk = std::min<std::size_t>(65535, raw.size() - pos);
    const bool final = (pos + blk == raw.size());
    zlib.push_back(final ? 1 : 0);
    zlib.push_back(static_cast<unsigned char>(blk & 0xFF));
    zlib.push_back(static_cast<unsigned char>((blk >> 8) & 0xFF));
    const std::uint16_t nlen = static_cast<std::uint16_t>(~blk);
    zlib.push_back(static_cast<unsigned char>(nlen & 0xFF));
    zlib.push_back(static_cast<unsigned char>((nlen >> 8) & 0xFF));
    zlib.insert(zlib.end(), raw.begin() + pos, raw.begin() + pos + blk);
    pos += blk;
    if (final)
    {
      break;
    }
  }
  std::uint32_t a = 1, b = 0;
  for (unsigned char c : raw)
  {
    a = (a + c) % 65521;
    b = (b + a) % 65521;
  }
  put_be32(zlib, (b << 16) | a);

  std::vector<unsigned char> out = {0x89, 'P',  'N',  'G',
                                    0x0D, 0x0A, 0x1A, 0x0A};
  std::vector<unsigned char> ihdr;
  put_be32(ihdr, w);
  put_be32(ihdr, h);
  ihdr.push_back(8);  // 8 bits/channel
  ihdr.push_back(6);  // color type 6 = RGBA
  ihdr.push_back(0);  // deflate
  ihdr.push_back(0);  // no filter
  ihdr.push_back(0);  // no interlace
  put_chunk(out, "IHDR", ihdr);
  put_chunk(out, "IDAT", zlib);
  put_chunk(out, "IEND", {});

  std::ofstream f(path, std::ios::binary);
  if (!f)
  {
    return false;
  }
  f.write(reinterpret_cast<const char*>(out.data()),
          static_cast<std::streamsize>(out.size()));
  return f.good();
}

}  // namespace

bool VulkanRenderer::capture(const glm::mat4& view, const glm::mat4& proj,
                             const std::string& path)
{
  if (!ok_ || !headless_)
  {
    return false;
  }
  const std::uint32_t w = sc_extent_.width, h = sc_extent_.height;
  const VkDeviceSize bytes = static_cast<VkDeviceSize>(w) * h * 4;

  // Host-visible buffer that the rendered image is copied into.
  if (readback_.capacity < bytes)
  {
    destroy_buffer(readback_);
    VkBufferCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    ci.size = bytes;
    ci.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    vkCreateBuffer(device_, &ci, nullptr, &readback_.buffer);
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device_, readback_.buffer, &req);
    VkMemoryAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = find_memory_type(
        req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    vkAllocateMemory(device_, &ai, nullptr, &readback_.memory);
    vkBindBufferMemory(device_, readback_.buffer, readback_.memory, 0);
    vkMapMemory(device_, readback_.memory, 0, VK_WHOLE_SIZE, 0,
                &readback_.mapped);
    readback_.capacity = req.size;
  }

  // Build a UI frame on the CPU (no GLFW backend: set the display size by
  // hand). Proof that the ImGui + Vulkan integration renders -- shows the demo
  // window.
  const bool draw_ui = imgui_ready_ && capture_ui_;
  if (draw_ui)
  {
    ImGui_ImplVulkan_NewFrame();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(w), static_cast<float>(h));
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    ImGui::ShowDemoWindow();
    ImGui::Render();
  }

  vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);
  vkResetFences(device_, 1, &fence_);
  vkResetCommandBuffer(cmd_, 0);

  VkCommandBufferBeginInfo bi{};
  bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cmd_, &bi);

  pipeline_barrier(
      cmd_, image_barrier(sc_images_[0], VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          VK_IMAGE_ASPECT_COLOR_BIT));
  pipeline_barrier(
      cmd_, image_barrier(depth_image_, VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                          VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                          VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                              VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                          VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                          VK_IMAGE_ASPECT_DEPTH_BIT));

  record_scene(0, proj * view);

  // ImGui overlay pass (color-only, load), same as the windowed path.
  if (draw_ui)
  {
    pipeline_barrier(
        cmd_,
        image_barrier(sc_images_[0], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                      VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT |
                          VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT,
                      VK_IMAGE_ASPECT_COLOR_BIT));
    VkRenderingAttachmentInfo ui_color{};
    ui_color.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    ui_color.imageView = sc_views_[0];
    ui_color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    ui_color.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    ui_color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    VkRenderingInfo ui_ri{};
    ui_ri.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    ui_ri.renderArea.extent = sc_extent_;
    ui_ri.layerCount = 1;
    ui_ri.colorAttachmentCount = 1;
    ui_ri.pColorAttachments = &ui_color;
    vkCmdBeginRendering(cmd_, &ui_ri);
    if (ImDrawData* dd = ImGui::GetDrawData())
    {
      ImGui_ImplVulkan_RenderDrawData(dd, cmd_);
    }
    vkCmdEndRendering(cmd_);
  }

  // Color attachment -> transfer source so we can copy it out.
  pipeline_barrier(
      cmd_,
      image_barrier(sc_images_[0], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                    VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                    VK_IMAGE_ASPECT_COLOR_BIT));

  VkBufferImageCopy region{};
  region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  region.imageExtent = {w, h, 1};
  vkCmdCopyImageToBuffer(cmd_, sc_images_[0],
                         VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, readback_.buffer,
                         1, &region);
  vkEndCommandBuffer(cmd_);

  VkCommandBufferSubmitInfo cmd_si{};
  cmd_si.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
  cmd_si.commandBuffer = cmd_;
  VkSubmitInfo2 submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
  submit.commandBufferInfoCount = 1;
  submit.pCommandBufferInfos = &cmd_si;
  vkQueueSubmit2(queue_, 1, &submit, fence_);
  vkWaitForFences(device_, 1, &fence_, VK_TRUE, UINT64_MAX);

  return write_png(path.c_str(), w, h,
                   static_cast<const unsigned char*>(readback_.mapped));
}

}  // namespace monitor
