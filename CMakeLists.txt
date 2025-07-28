cmake_minimum_required(VERSION 3.20)


message(STATUS "Updating Git submodules...")
message(STATUS "dir: ${CMAKE_SOURCE_DIR}")
execute_process(
    COMMAND git submodule update --remote --merge
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE result
)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Git submodule update failed with result: ${result}")
endif()

project(dma_baremetal C ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Source files
set(C_SOURCES
    src/main.c
    src/buffer.c
    src/dma.c
    src/gpio.c
    src/usart.c
    src/rcc.c
    cmsis/system_stm32f3xx.c)

set(ASM_SOURCES
    startup_stm32f303xe.s)

add_subdirectory(external/bitmap_allocator)

# Executable target
add_executable(${PROJECT_NAME}.elf
    ${C_SOURCES}
    ${ASM_SOURCES})

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
target_compile_options(bitmap_allocator PRIVATE ${COMMON_C_FLAGS} ${MCU_FLAGS})  
target_link_libraries(${PROJECT_NAME}.elf PRIVATE bitmap_allocator)

# Post-build: generate HEX and BIN, show size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE_UTIL} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Generating .hex/.bin and displaying size")

# Clean up extra files if `make clean`
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES
    "${PROJECT_NAME}.hex" "${PROJECT_NAME}.bin" "${PROJECT_NAME}.map")
