cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME buck_boost_closeloop)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ---------------------------------------------------------------------------
# 模块源码
# ---------------------------------------------------------------------------
# 用法：把依赖的模块 .h 复制到 Core/Inc、.c 复制到 Core/Src，重新构建即可。
# Core/Inc 已由 cmake/stm32cubemx 加入 include 路径，这里只处理 Core/Src。
# CubeMX 生成的那几个文件已在该子目录中列出，需从 GLOB 结果里排除以免重复。
set(MX_GENERATED_SRC
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/main.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/stm32g4xx_it.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/stm32g4xx_hal_msp.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/sysmem.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/syscalls.c
    ${CMAKE_CURRENT_LIST_DIR}/Core/Src/system_stm32g4xx.c
)

file(GLOB MODULE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/Core/Src/*.c)
list(REMOVE_ITEM MODULE_SRC ${MX_GENERATED_SRC})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MODULE_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Core/Inc 已由 stm32cubemx 提供
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# ---------------------------------------------------------------------------
# newlib-nano 的浮点 printf
# ---------------------------------------------------------------------------
# --specs=nano.specs 默认不支持 %f，而 OLED 显示和串口遥测都要打印小数，
# 所以这里显式打开，代价是代码体积增加约 1.5~3KB。
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
