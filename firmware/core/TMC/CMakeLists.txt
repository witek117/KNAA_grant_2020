file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp )

add_library(TMC ${SOURCES})
target_include_directories(TMC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/)
#target_include_directories(TMC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/source/)

# for Stream
target_include_directories(TMC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)

