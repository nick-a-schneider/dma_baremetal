cmake_minimum_required(VERSION 3.20)

include(FetchContent)

if(NOT TARGET buffers)
    message(STATUS "Fetching buffers...")
    FetchContent_Declare(
        buffers
        GIT_REPOSITORY https://github.com/nick-a-schneider/buffers.git
        GIT_TAG master
    )
    FetchContent_MakeAvailable(buffers)
endif()
if(NOT TARGET buffers)
    message(FATAL_ERROR "buffers not found")
endif()

project(dma_baremetal C ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Source files
set(C_SOURCES
    src/main.c
    src/dma.c
    src/gpio.c
    src/usart.c
    src/rcc.c
    cmsis/system_stm32f3xx.c)

set(ASM_SOURCES
    startup_stm32f303xe.s)

# Executable target
add_executable(${PROJECT_NAME}.elf
    ${C_SOURCES}
    ${ASM_SOURCES})

target_link_libraries(${PROJECT_NAME}.elf PRIVATE
    buffers)

# Include directories
target_include_directories(${PROJECT_NAME}.elf PRIVATE
    inc
    cmsis)
    
# Preprocessor defines
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    STM32F303xE)

# Common compile flags for all targets
set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

set(COMMON_C_FLAGS
    -O0
    -Wall
    -fdata-sections
    -ffunction-sections
    -g
    -gdwarf-2
)

# Linker script
set(LDSCRIPT ${CMAKE_SOURCE_DIR}/STM32F303RETx_FLASH.ld)

# Common linker flags
set(COMMON_LINK_FLAGS
    ${MCU_FLAGS}
    -specs=nano.specs
    -T${LDSCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -lc -lm -lnosys
)

# Apply to main target
target_compile_options(${PROJECT_NAME}.elf PRIVATE ${COMMON_C_FLAGS} ${MCU_FLAGS})
target_link_options(${PROJECT_NAME}.elf PRIVATE ${COMMON_LINK_FLAGS} ${MCU_FLAGS})

# Also apply to the allocator target
target_compile_options(buffers PRIVATE ${COMMON_C_FLAGS} ${MCU_FLAGS})  
target_link_libraries(${PROJECT_NAME}.elf PRIVATE buffers)

# Post-build: generate HEX and BIN, show size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE_UTIL} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating .hex/.bin and displaying size")

# Clean up extra files if `make clean`
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${PROJECT_NAME}.hex" "${PROJECT_NAME}.bin" "${PROJECT_NAME}.map")
