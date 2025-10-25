# my-raytracer/CMakeLists.txt
file(GLOB_RECURSE SOL_SRCS CONFIGURE_DEPENDS ./*.cpp)

# Add solution files into the existing raytracer executable
target_sources(raytracer PRIVATE ${SOL_SRCS})

# Make sure headers from src/ are visible when compiling solutions/
target_include_directories(raytracer PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
