cmake_minimum_required(VERSION 3.16)
project(neato_nav LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
  add_compile_options(/W4 /permissive-)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core library: everything except the small programs in apps/
add_library(neato_core
  src/serial_port.cpp
  src/neato_protocol.cpp
  src/real_robot.cpp
  src/sim_robot.cpp
  src/world.cpp
  src/odometry.cpp
  src/drive_log.cpp
)
target_include_directories(neato_core PUBLIC include)

# Talks to the real robot over USB (works now, on dock power)
add_executable(battery_monitor apps/battery_monitor.cpp)
target_link_libraries(battery_monitor neato_core)

# Simulated Neato in the apartment (no hardware needed), writes sim_log.json
add_executable(apartment_sim apps/apartment_sim.cpp)
target_link_libraries(apartment_sim neato_core)

# Keyboard driving with recording, real robot or --sim
add_executable(teleop apps/teleop.cpp)
target_link_libraries(teleop neato_core)

# Experiment: which serial command stops the wheels mid-move (docs/calibration.md)
add_executable(stop_test apps/stop_test.cpp)
target_link_libraries(stop_test neato_core)

# Unit tests for the text parsers
enable_testing()
add_executable(test_parsers tests/test_parsers.cpp)
target_link_libraries(test_parsers neato_core)
add_test(NAME parsers COMMAND test_parsers)

# Recorder test: drives the simulator and writes a log that CI plots
add_executable(test_drive_log tests/test_drive_log.cpp)
target_link_libraries(test_drive_log neato_core)
add_test(NAME drive_log
         COMMAND test_drive_log ${CMAKE_SOURCE_DIR}/maps/apartment.map ${CMAKE_BINARY_DIR}/drive_log_test.jsonl)
