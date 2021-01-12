
add_library(VL53L0X VL53L0X.cpp)
target_include_directories(VL53L0X PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/)

# for I2C and HAL
target_include_directories(VL53L0X PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)