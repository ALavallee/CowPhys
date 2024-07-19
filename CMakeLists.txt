cmake_minimum_required(VERSION 3.28)
project(CowPhys)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG master  # Specify the version you want to use
)
FetchContent_MakeAvailable(raylib)
include_directories(${raylib_SOURCE_DIR}/include)

file(GLOB_RECURSE COW_PHYS_SOURCES
        "src/*.c"
        "src/*.h"
        "src/*.cpp"
        "src/*.hpp"
)


add_executable(CowPhys ${COW_PHYS_SOURCES})

target_include_directories(CowPhys PRIVATE src)

target_link_libraries(CowPhys PRIVATE raylib)