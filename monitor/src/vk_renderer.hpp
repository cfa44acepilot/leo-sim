/*****************************************************************************
  filename vk_renderer.hpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    A small Vulkan 1.4 renderer for the LEO monitor.

    Vulkan 1.4 throughout: dynamic rendering (no VkRenderPass objects) and
    synchronization2 (VkImageMemoryBarrier2 / vkQueueSubmit2). ONE frame in
    flight, deliberately -- this is a visualizer, not a throughput benchmark,
    and one frame keeps the synchronization obviously correct rather than
    merely probably correct. Every buffer is host-visible and persistently
    mapped: the geometry is small (a globe plus ~10k points), so a staging path
    would buy nothing but code to get wrong.

    The renderer knows NOTHING about the simulator. It is handed plain vertex
    arrays built by scene.cpp, which keeps the core <-> render boundary one-way:
    the simulator never learns that it is being drawn.
 *****************************************************************************/

#ifndef LEO_MONITOR_VK_RENDERER_HPP
#define LEO_MONITOR_VK_RENDERER_HPP

#include <cstdint> /* fixed-width sizes for Vulkan's structs       */
#include <string>  /* texture and screenshot paths                 */
#include <vector>  /* the geometry handed in from scene.cpp        */

#include <vulkan/vulkan.h> /* device, pipelines, dynamic rendering */

#include <glm/mat4x4.hpp> /* view/proj matrices from the camera    */
#include <glm/vec3.hpp>   /* point geometry (satellites, markers)  */

#include "scene.hpp" /* MeshVertex / LineVertex / RouteVertex      */

struct GLFWwindow; /* forward-declared: GLFW is included only in the .cpp */

namespace monitor
{

class VulkanRenderer
{
 public:
  explicit VulkanRenderer(GLFWwindow* window);

  /* Headless constructor: render to an offscreen target of the given size, with
     NO window and no swapchain. This is what makes --screenshot work in a
     non-interactive session -- no visible desktop, no window to be denied the
     foreground -- and it is therefore the ONLY way the renderer can be verified
     from an automated run. */
  VulkanRenderer(int width, int height);
  ~VulkanRenderer();

  VulkanRenderer(const VulkanRenderer&) = delete;
  VulkanRenderer& operator=(const VulkanRenderer&) = delete;

  /* True only if every Vulkan object came up. Construction never throws, so a
     missing GPU or driver becomes a clean exit rather than a crash. */
  bool ok() const { return ok_; }

  /* Static geometry: uploaded once at startup. */
  void set_earth_mesh(const std::vector<MeshVertex>& v,     /* vertices */
                      const std::vector<std::uint32_t>& idx /* indices  */
  );
  void set_atmosphere_mesh(const std::vector<MeshVertex>& v,     /* vertices */
                           const std::vector<std::uint32_t>& idx /* indices  */
  );

  /* Dynamic geometry, re-uploaded by the caller when it changes. Points and
     edges move every sim tick; so does the route, because a cached route line
     drifts off the satellites it is supposed to run through. */
  void set_points(const std::vector<glm::vec3>& pts);
  void set_edges(const std::vector<LineVertex>& lines);
  void set_path(const std::vector<RouteVertex>& verts);

  /* The selected satellite's world position (one element), or empty to clear
     the stencil-outline highlight. Re-uploaded each frame so the ring tracks
     the satellite as it moves. */
  void set_selection(const std::vector<glm::vec3>& pt);

  /* World positions of blind ground endpoints (0..2) to ring in amber, marking
     where the network has no coverage at all. Empty clears them. */
  void set_markers(const std::vector<glm::vec3>& pts);

  /* Render one frame. view/proj come from the orbit camera. */
  void draw(const glm::mat4& view, const glm::mat4& proj);

  /* Headless only: render one frame to the offscreen target and write it out as
     a PNG. Returns false on failure. */
  bool capture(const glm::mat4& view,  /* camera        */
               const glm::mat4& proj,  /* projection    */
               const std::string& path /* PNG to write  */
  );

  /* Block until the GPU is idle -- call before destroying anything the GPU may
     still be reading. */
  void wait_idle();

  /* Headless only: overlay the ImGui demo on a captured frame, as proof the UI
     pass runs. Off by default, so a normal --screenshot stays globe-only. */
  void set_capture_ui(bool on) { capture_ui_ = on; }

 private:
  /* A host-visible, persistently mapped buffer that grows on demand. Mapping it
     once and leaving it mapped costs nothing here and removes a whole class of
     map/unmap lifetime bugs. */
  struct Buffer
  {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    void* mapped = nullptr;    /* persistently mapped: never unmapped early  */
    VkDeviceSize capacity = 0; /* bytes; grows, never shrinks                */
    std::uint32_t count = 0;   /* element count last uploaded                */
  };

  struct VertexInput
  {
    std::vector<VkVertexInputBindingDescription> bindings;
    std::vector<VkVertexInputAttributeDescription> attrs;
  };

  /* --- setup steps; each returns false on failure so ok_ can gate the run ---
   */
  bool create_instance();
  bool create_surface(GLFWwindow* window);
  bool pick_device();
  bool create_device();
  bool create_swapchain();
  void destroy_swapchain();
  bool recreate_swapchain();
  bool create_offscreen_target(); /* headless: one color image, no swapchain */
  bool create_depth();
  bool create_commands_and_sync();
  bool create_pipelines();
  void init_imgui(GLFWwindow* window); /* windowed only; best-effort */

  VkShaderModule load_shader(const char* name);

  /*-------------------------------------------------------------------------
    Function: make_pipeline
    Description: The one pipeline factory every pass goes through, so a new pass
                 differs from the others only in the ways it MEANS to.

                 color_write and the stencil state exist for the selection
                 outline: the mask pass writes stencil only, and the outline
                 pass draws only where the stencil is clear -- which is what
                 turns a disc into a ring. Every other pipeline takes the
                 defaults (color on, stencil off) and is unaffected.
    Input: vert, frag     -- shader module names
           topo           -- primitive topology
           vi             -- vertex bindings/attributes
           depth_test     -- test against the shared depth buffer
           depth_write    -- write depth (off for overlays and translucency)
           blend          -- alpha blending
           cull           -- cull mode
           layout         -- pipeline layout (push constants, +/- descriptors)
           color_write    -- false for a stencil-only pass
           stencil_enable -- turn the stencil test on
           stencil        -- the stencil op state when it is on
    Outputs: The pipeline, or VK_NULL_HANDLE on failure.
  -------------------------------------------------------------------------*/
  VkPipeline make_pipeline(
      const char* vert,             /* vertex shader name             */
      const char* frag,             /* fragment shader name           */
      VkPrimitiveTopology topo,     /* triangles / lines / points     */
      const VertexInput& vi,        /* bindings + attributes          */
      bool depth_test,              /* test against the depth buffer  */
      bool depth_write,             /* write depth                    */
      bool blend,                   /* alpha blending                 */
      VkCullModeFlags cull,         /* which faces to cull            */
      VkPipelineLayout layout,      /* push constants (+ descriptors) */
      bool color_write = true,      /* false = stencil-only pass      */
      bool stencil_enable = false,  /* the selection outline needs it */
      VkStencilOpState stencil = {} /* its op state when enabled      */
  );

  /* Earth day-map texture: descriptor set layout + sampler + image + upload.
     load_earth_texture falls back to a 1x1 white texture when the file is
     missing, so the earth pipeline ALWAYS has a valid descriptor bound and a
     missing asset degrades the picture instead of breaking the run. */
  bool create_texture_infra();
  bool load_earth_texture(const std::string& path);
  bool upload_texture_rgba(
      const unsigned char* rgba, /* tightly packed RGBA8 pixels */
      int w,                     /* width                       */
      int h                      /* height                      */
  );

  std::uint32_t find_memory_type(
      std::uint32_t type_bits,    /* candidate memory types from Vulkan */
      VkMemoryPropertyFlags props /* what we need of them               */
  ) const;

  void upload(Buffer& b,                /* grown in place if it is too small */
              const void* data,         /* source bytes                      */
              VkDeviceSize bytes,       /* how many                          */
              VkBufferUsageFlags usage, /* vertex or index                   */
              std::uint32_t count       /* element count, for the draw call  */
  );
  void destroy_buffer(Buffer& b);

  void record(std::uint32_t image_index, /* swapchain image to render into */
              const glm::mat4& view_proj /* the camera, as one matrix      */
  );

  /* The shared scene draw (begin_rendering + the passes + end_rendering), used
     by BOTH the windowed present path and the headless capture path -- so a
     screenshot is a true witness of what the window shows, not a second
     rendering that could quietly diverge from it. */
  void record_scene(std::uint32_t image_index, /* target image */
                    const glm::mat4& view_proj /* the camera   */
  );

  GLFWwindow* window_ = nullptr; /* null in headless mode           */
  bool headless_ = false;        /* no swapchain, no presentation   */
  bool ok_ = false;              /* every object came up            */

  VkInstance instance_ = VK_NULL_HANDLE;
  VkSurfaceKHR surface_ = VK_NULL_HANDLE;
  VkPhysicalDevice phys_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;
  std::uint32_t queue_family_ = 0; /* one family does graphics + present */
  VkQueue queue_ = VK_NULL_HANDLE;

  VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
  VkFormat sc_format_ = VK_FORMAT_UNDEFINED;
  VkExtent2D sc_extent_ = {0, 0};
  std::vector<VkImage> sc_images_;
  std::vector<VkImageView> sc_views_;

  /* Headless only: WE own this color image (a swapchain owns its own). */
  VkImage offscreen_image_ = VK_NULL_HANDLE;
  VkDeviceMemory offscreen_memory_ = VK_NULL_HANDLE;

  /* Depth + STENCIL. The stencil aspect is what drives the selection outline;
     D32S8 is ubiquitous on desktop GPUs, so no format-fallback query is worth
     the code here. */
  VkFormat depth_format_ = VK_FORMAT_D32_SFLOAT_S8_UINT;
  VkImage depth_image_ = VK_NULL_HANDLE;
  VkDeviceMemory depth_memory_ = VK_NULL_HANDLE;
  VkImageView depth_view_ = VK_NULL_HANDLE;

  VkCommandPool cmd_pool_ = VK_NULL_HANDLE;
  VkCommandBuffer cmd_ = VK_NULL_HANDLE;
  VkSemaphore sem_acquire_ = VK_NULL_HANDLE;
  VkSemaphore sem_render_ = VK_NULL_HANDLE;
  VkFence fence_ = VK_NULL_HANDLE;

  VkPipelineLayout pipe_layout_ = VK_NULL_HANDLE;  /* push constants only    */
  VkPipelineLayout earth_layout_ = VK_NULL_HANDLE; /* + texture descriptor   */
  VkPipeline pipe_earth_ = VK_NULL_HANDLE;
  VkPipeline pipe_atmo_ = VK_NULL_HANDLE;
  VkPipeline pipe_points_ = VK_NULL_HANDLE;
  VkPipeline pipe_lines_ = VK_NULL_HANDLE;
  VkPipeline pipe_route_ = VK_NULL_HANDLE;       /* thick, always-on-top     */
  VkPipeline pipe_sel_mask_ = VK_NULL_HANDLE;    /* selection: writes stencil */
  VkPipeline pipe_sel_outline_ = VK_NULL_HANDLE; /* selection: draws the ring */
  VkPipeline pipe_marker_ = VK_NULL_HANDLE;      /* blind-endpoint amber ring */

  /* Earth albedo texture and its descriptor. */
  VkDescriptorSetLayout tex_set_layout_ = VK_NULL_HANDLE;
  VkDescriptorPool tex_pool_ = VK_NULL_HANDLE;
  VkDescriptorSet tex_set_ = VK_NULL_HANDLE;
  VkSampler tex_sampler_ = VK_NULL_HANDLE;
  VkImage tex_image_ = VK_NULL_HANDLE;
  VkDeviceMemory tex_memory_ = VK_NULL_HANDLE;
  VkImageView tex_view_ = VK_NULL_HANDLE;

  Buffer earth_vb_, earth_ib_;
  Buffer atmo_vb_, atmo_ib_;
  Buffer points_vb_;
  Buffer edges_vb_;
  Buffer path_vb_;
  Buffer sel_vb_;    /* the selected satellite (0 or 1 point) */
  Buffer marker_vb_; /* blind ground endpoints (0..2 points)  */

  Buffer readback_; /* headless: the offscreen image lands here for encoding */

  /* Dear ImGui, windowed path only. imgui_ready_ gates EVERY ImGui call, so the
     headless path and a failed UI init are both safe no-ops rather than crashes
     in a library that was never initialized. */
  VkDescriptorPool imgui_pool_ = VK_NULL_HANDLE;
  bool imgui_ready_ = false;
  bool capture_ui_ = false; /* headless: draw the demo overlay in capture() */
};

} /* namespace monitor */

#endif /* LEO_MONITOR_VK_RENDERER_HPP */
