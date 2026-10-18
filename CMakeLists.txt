cmake_minimum_required(VERSION 3.21)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi-gcc.cmake"
        CACHE FILEPATH "STM32 GCC toolchain")
endif()
project(F405_FlightControl LANGUAGES C ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/sources.cmake)
add_executable(F405 ${F405_SOURCES} GCC/startup_stm32f405xx.s GCC/syscalls.c)
set_target_properties(F405 PROPERTIES SUFFIX ".elf")
set(_cpu -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_definitions(F405 PRIVATE USE_HAL_DRIVER STM32F405xx VECT_TAB_OFFSET=0x10000U)
target_compile_options(F405 PRIVATE ${_cpu}
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections;-fdata-sections;-fno-common;-Wall;-Wextra;-Wno-unused-parameter>
    $<$<CONFIG:Debug>:-O0;-g3;-gdwarf-3;-gstrict-dwarf>
    $<$<CONFIG:Release>:-Os;-g0>)
# Preserve the existing header spellings without changing shared Keil sources.
# These generated forwarding headers also support case-sensitive filesystems.
set(_compat "${CMAKE_CURRENT_BINARY_DIR}/compat_include")
file(MAKE_DIRECTORY "${_compat}")
file(CONFIGURE OUTPUT "${_compat}/Bsp_LC307.h" CONTENT "#include \"${CMAKE_CURRENT_SOURCE_DIR}/F405_BSP/Inc/Bsp_lc307.h\"\n" @ONLY)
file(CONFIGURE OUTPUT "${_compat}/bsp_ps2.h" CONTENT "#include \"${CMAKE_CURRENT_SOURCE_DIR}/F405_BSP/Inc/Bsp_ps2.h\"\n" @ONLY)
file(CONFIGURE OUTPUT "${_compat}/bsp_adc.h" CONTENT "#include \"${CMAKE_CURRENT_SOURCE_DIR}/F405_BSP/Inc/Bsp_adc.h\"\n" @ONLY)
target_include_directories(F405 PRIVATE "${_compat}" ${F405_INCLUDE_DIRS})
set(_ld "${CMAKE_CURRENT_SOURCE_DIR}/GCC/STM32F405RG_APP.ld")
set_property(TARGET F405 APPEND PROPERTY LINK_DEPENDS "${_ld}")
target_link_options(F405 PRIVATE ${_cpu} -nostartfiles --specs=nano.specs --specs=nosys.specs
    "-T${_ld}" "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/F405.map,--cref"
    -Wl,--gc-sections,--print-memory-usage -Wl,-u,_printf_float)
target_link_libraries(F405 PRIVATE m)
add_custom_command(TARGET F405 POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary "$<TARGET_FILE:F405>" "${CMAKE_CURRENT_BINARY_DIR}/F405.bin"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:F405>" "${CMAKE_CURRENT_BINARY_DIR}/F405.hex"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:F405>"
    VERBATIM)
