cmake_minimum_required(VERSION 3.16)
project(sensor_fusion_navigation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(nav_core
  src/Config.cpp
  src/Types.cpp
  src/SensorFilter.cpp
  src/FusionEngine.cpp
  src/DecisionEngine.cpp
)
target_include_directories(nav_core PUBLIC include)
target_compile_options(nav_core PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(nav_tests test/native/test_navigation.cpp)
target_link_libraries(nav_tests PRIVATE nav_core)
target_compile_options(nav_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME nav_tests COMMAND nav_tests)
