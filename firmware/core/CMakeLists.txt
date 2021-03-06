add_subdirectory(external)
add_subdirectory(VL53L0X)
add_subdirectory(TMC)

set(ONLY_LIBRARY "ONLY_LIBRARY")
add_subdirectory(CommandParser)

add_library(core core.cpp ring_buffer.cpp)

target_link_libraries(core PUBLIC msftgsl VL53L0X TMC CommandParser)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
