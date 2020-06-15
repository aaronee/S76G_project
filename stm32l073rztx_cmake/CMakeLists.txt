#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.7)

# specify cross compilers and tools
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_C_COMPILER arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(EXECUTABLE_OUTPUT_PATH build)
set(CMAKE_BUILD_PARALLEL_LEVEL 12)

SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L073RZTx_FLASH.ld)

#Uncomment for hardware floating point
#SET(FPU_FLAGS "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
#add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)

#Uncomment for software floating point
#SET(FPU_FLAGS "-mfloat-abi=soft")

SET(COMMON_FLAGS
    "-mcpu=cortex-m0 ${FPU_FLAGS} -mthumb -mthumb-interwork -ffunction-sections -fdata-sections \
    -g -fno-common -fmessage-length=0 -specs=nosys.specs -specs=nano.specs")

SET(CMAKE_CXX_FLAGS_INIT "${COMMON_FLAGS} -std=c++11")
SET(CMAKE_C_FLAGS_INIT "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,-gc-sections,--print-memory-usage -T ${LINKER_SCRIPT}")

PROJECT(stm32l073rztx_cmake C CXX ASM)
set(CMAKE_CXX_STANDARD 11)

#add_definitions(-DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING -D__FPU_PRESENT=1)
add_definitions(-DUSE_HAL_DRIVER -DSTM32L073xx -DACTIVE_REGION=LORAMAC_REGION_US915 -DREGION_US915 -DLORA_USE_TCXO) #-DLORA_USE_TCXO

file(GLOB_RECURSE SOURCES "startup/*.*" "Drivers/*.*" "Src/*.*" "Middlewares/*.*")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/LoRaMac/classA/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/LoRaMac/classB/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/LoRaMac/classC/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/LoRaMac/fuota-test-01/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/ping-pong/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/rx-sensi/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/tx-cw/NucleoL073/main.c"
                         "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/LoRaMac-node/apps/LoRaMac/periodic-uplink-lpp/NucleoL073/main.c"
                         )

include_directories(Inc
        Drivers/STM32L0xx_HAL_Driver/Inc
        Drivers/STM32L0xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32L0xx/Include
        Drivers/CMSIS/Include
        Middlewares/LoRaMac-node/apps/LoRaMac/common/LmHandler/packages
        Middlewares/LoRaMac-node/apps/LoRaMac/common/LmHandler
        Middlewares/LoRaMac-node/apps/LoRaMac/common
        Middlewares/LoRaMac-node/boards
        Middlewares/LoRaMac-node/boards/S76G-lilygo-tmotion
        Middlewares/LoRaMac-node/mac/region
        Middlewares/LoRaMac-node/mac
        Middlewares/LoRaMac-node/peripherals
        Middlewares/LoRaMac-node/peripherals/soft-se
        Middlewares/LoRaMac-node/radio/sx1276
        Middlewares/LoRaMac-node/radio
        Middlewares/LoRaMac-node/system
        Middlewares/LoRaMac-node/system/crypto)

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${PROJECT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.map")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
