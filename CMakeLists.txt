cmake_minimum_required(VERSION 3.23)

project(HybridAStarHtmlDemo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(FLTK_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_FLUID OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_FLTK_OPTIONS OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_FORMS OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_GL OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_HTML_DOCS OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_PDF_DOCS OFF CACHE BOOL "" FORCE)
set(FLTK_BUILD_FLUID_DOCS OFF CACHE BOOL "" FORCE)

add_subdirectory(third_party/fltk-1.4.5)

file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_executable(hybrid_astar
    ${PROJECT_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl_reeds_shepp/OmplReedsShepp.cpp"
)

target_include_directories(hybrid_astar
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl_reeds_shepp"
)

target_link_libraries(hybrid_astar
    PRIVATE
    fltk::fltk
)

if(MSVC)
    target_compile_options(hybrid_astar PRIVATE /W4)
else()
    target_compile_options(hybrid_astar PRIVATE -Wall -Wextra -Wpedantic)
endif()
