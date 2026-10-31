cmake_minimum_required(VERSION 3.20)
project(spatial_map_foundation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# 空间状态运行时（fork-spatial 核心）
add_library(fork_spatial_runtime STATIC
  src/runtime/spatial_runtime.cpp
  src/runtime/state_store.cpp
)
target_include_directories(fork_spatial_runtime PUBLIC src)

# 哈希瓦片地图底座（header-only）
add_library(spatial_map INTERFACE)
target_include_directories(spatial_map INTERFACE src)
target_link_libraries(spatial_map INTERFACE fork_spatial_runtime)

# 组合演示
add_executable(map_state_demo demo/map_state_demo.cpp)
target_link_libraries(map_state_demo PRIVATE spatial_map)

# 高德瓦片工具
add_executable(amap_tile_tool tools/amap_tile_tool.cpp)
target_link_libraries(amap_tile_tool PRIVATE spatial_map)

# 单元测试
enable_testing()
add_executable(map_state_test tests/map_state_test.cpp)
target_link_libraries(map_state_test PRIVATE spatial_map)
add_test(NAME map_state_test COMMAND map_state_test)

add_executable(amap_tile_test tests/amap_tile_test.cpp)
target_link_libraries(amap_tile_test PRIVATE spatial_map)
add_test(NAME amap_tile_test COMMAND amap_tile_test)
