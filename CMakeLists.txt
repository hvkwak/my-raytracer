file(GLOB SOL_SRCS CONFIGURE_DEPENDS ./*.cpp)
file(GLOB SOL_HDRS CONFIGURE_DEPENDS ./*.h)

# Conditionally add cuda subdirectory if CUDA is available
if(CUDAToolkit_FOUND)
  file(GLOB_RECURSE CUDA_SRCS CONFIGURE_DEPENDS ./cuda/*.cu)
  file(GLOB_RECURSE CUDA_HDRS CONFIGURE_DEPENDS ./cuda/*.h)
  # Add solution files into the existing raytracer executable
  target_sources(raytracer PRIVATE ${SOL_SRCS} ${SOL_HDRS} ${CUDA_SRCS} ${CUDA_HDRS})
else()
  target_sources(raytracer PRIVATE ${SOL_SRCS} ${SOL_HDRS})
endif()

# Make sure headers from src/ are visible when compiling solutions/
target_include_directories(raytracer PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
