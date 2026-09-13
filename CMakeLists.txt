cmake_minimum_required(VERSION 3.22)

set(CMAKE_PROJECT_NAME Vehicle_Target)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${CMAKE_PROJECT_NAME})

# STM32CubeMX generated startup, HAL, CMSIS and FreeRTOS sources.
add_subdirectory(cmake/stm32cubemx)

# Application and reusable user modules.
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.S"
)

# A newer chassis FSM lives in User/Module/Chassis. Do not link the legacy
# implementation with the same class and symbols.
list(FILTER USER_SOURCES EXCLUDE REGEX
    ".*/User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis\\.cpp$"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# The project uses short header names across the User tree, so expose every
# directory that contains a C/C++ header.
file(GLOB_RECURSE USER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/User/*.hpp"
)

set(USER_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/User")
foreach(USER_HEADER IN LISTS USER_HEADERS)
    get_filename_component(USER_HEADER_DIR "${USER_HEADER}" DIRECTORY)
    list(APPEND USER_INCLUDE_DIRS "${USER_HEADER_DIR}")
endforeach()
list(REMOVE_DUPLICATES USER_INCLUDE_DIRS)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# Force the final link through the C++ driver when C++ application files exist.
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINKER_LANGUAGE CXX
)

# Generate files suitable for programmers and debugging tools.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating HEX and BIN firmware images"
)
