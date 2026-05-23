cmake_minimum_required(VERSION 3.23)

project(HybridAStarHtmlDemo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

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

add_executable(reeds_shepp_empty_map_test
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/reeds_shepp_empty_map_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/Car.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CollisionChecker.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GridMap.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ReedsShepp.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl_reeds_shepp/OmplReedsShepp.cpp"
)

target_include_directories(reeds_shepp_empty_map_test
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl_reeds_shepp"
)

target_compile_definitions(reeds_shepp_empty_map_test
    PRIVATE
    HYBRID_ASTAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

add_test(NAME reeds_shepp_empty_map_test
         COMMAND reeds_shepp_empty_map_test)

add_executable(line_of_sight_test
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/line_of_sight_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/LineOfSight.cpp"
)

target_include_directories(line_of_sight_test
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_test(NAME line_of_sight_test
         COMMAND line_of_sight_test)

add_executable(app_config_test
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/app_config_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/AppConfig.cpp"
)

target_include_directories(app_config_test
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_test(NAME app_config_test
         COMMAND app_config_test)

add_executable(path_json_viewer
    "${CMAKE_CURRENT_SOURCE_DIR}/tool/path_json_viewer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/FltkCanvas.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/FltkViewer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/GridMap.cpp"
)

target_include_directories(path_json_viewer
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_definitions(path_json_viewer
    PRIVATE
    HYBRID_ASTAR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(path_json_viewer
    PRIVATE
    fltk::fltk
)

if(MSVC)
    target_compile_options(hybrid_astar PRIVATE /W4)
    target_compile_options(reeds_shepp_empty_map_test PRIVATE /W4)
    target_compile_options(line_of_sight_test PRIVATE /W4)
    target_compile_options(app_config_test PRIVATE /W4)
    target_compile_options(path_json_viewer PRIVATE /W4)
else()
    target_compile_options(hybrid_astar PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(reeds_shepp_empty_map_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(line_of_sight_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(app_config_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(path_json_viewer PRIVATE -Wall -Wextra -Wpedantic)
endif()
