file(GLOB SOL_SRCS CONFIGURE_DEPENDS ./*.cpp)
file(GLOB SOL_HDRS CONFIGURE_DEPENDS ./*.h)
file(GLOB CUDA_SRCS CONFIGURE_DEPENDS ./*.cu)

# Add solution files into the existing raytracer executable
target_sources(raytracer PRIVATE ${SOL_SRCS} ${SOL_HDRS} ${CUDA_SRCS})

# Make sure headers from src/ are visible when compiling solutions/
target_include_directories(raytracer PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
