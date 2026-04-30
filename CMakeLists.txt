cmake_minimum_required(VERSION 3.23)

project(HybridAStarHtmlDemo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(hybrid_astar
    ${PROJECT_SOURCES}
)

target_include_directories(hybrid_astar
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if(MSVC)
    target_compile_options(hybrid_astar PRIVATE /W4)
else()
    target_compile_options(hybrid_astar PRIVATE -Wall -Wextra -Wpedantic)
endif()
