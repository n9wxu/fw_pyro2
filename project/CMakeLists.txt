cmake_minimum_required(VERSION 3.25)

# import the pico-sdk
set(ENV{PICO_SDK_FETCH_FROM_GIT} 1)
include(pico_sdk_import.cmake)

include(FetchContent)

FetchContent_Declare(libgps
    GIT_REPOSITORY https://github.com/n9wxu/libgps.git
)

FetchContent_MakeAvailable(libgps)
add_subdirectory(${libgps_SOURCE_DIR}/src)

FetchContent_Declare(lsm6ds3tr
    GIT_REPOSITORY https://github.com/STMicroelectronics/LSM6DS3TR-C-PID.git
)

FetchContent_MakeAvailable(lsm6ds3tr)

FetchContent_Declare(freertos
    GIT_REPOSITORY https://github.com/freertos/freertos-kernel.git
    GIT_TAG V11.0.1)

add_library(freertos_config INTERFACE)
set(FREERTOS_PORT GCC_RP2040)
FetchContent_Populate(freertos)
set(FREERTOS_KERNEL_PATH ${freertos_SOURCE_DIR})
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)


message(VERSION_REVISION="${PICO_SDK_VERSION_REVISION}")

project(pyro2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_definitions(-DPICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=5000)

add_executable(${CMAKE_PROJECT_NAME}
    main.c
    gps_task.c
    i2c_support.c
    sensors.c
    spi_drv.c
    pio_spi.c
    ${lsm6ds3tr_SOURCE_DIR}/lsm6ds3tr-c_reg.c
    )

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR} 
    ${lsm6ds3tr_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_pio
    hardware_rtc
    hardware_uart
    hardware_gpio
    hardware_i2c
    hardware_spi
    hardware_adc
    libgps
)

pico_add_extra_outputs(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
