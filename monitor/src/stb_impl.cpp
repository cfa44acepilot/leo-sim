/*****************************************************************************
  filename stb_impl.cpp
  author Tarrant Mangasarian
  Project Name: LEO constellation simulator (monitor)
  date 2026-07-14
  Brief Description:
    The single translation unit that compiles stb_image. Kept separate so the
    (large) implementation is built once and does not recompile every time
    vk_renderer.cpp changes; everything else includes "stb_image.h" for the
    declarations alone.
 *****************************************************************************/

/* The one define that turns the header into the implementation. This is a
   genuine preprocessor switch stb reads, not a typed constant, so it is the
   #define the style permits. It MUST precede the include. */
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"  /* JPEG/PNG decode for the Earth albedo texture */
