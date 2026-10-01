# ESP32-C3 蓝牙串口调试器 (BLE UART / NUS 桥接)
#
# 复用 ESP-IDF 官方 ble_uart_service 例程里的 common/ble_uart 组件，
# 它实现了事实标准 NUS(Nordic UART Service) 的 GATT 布局。
#
# 工程只写桥接逻辑：BLE <-> UART(TTL) <-> 被测设备。

cmake_minimum_required(VERSION 3.22)

# 把 IDF 例程自带的 ble_uart 组件目录加入搜索路径，这样 main 就能
# REQUIRES ble_uart，而无需拷贝组件源码进本工程。
file(TO_CMAKE_PATH "$ENV{IDF_PATH}/examples/bluetooth/common/ble_uart" BLE_UART_COMPONENT_DIR)
list(APPEND EXTRA_COMPONENT_DIRS "${BLE_UART_COMPONENT_DIR}")

# 官方例程同款：蓝牙场景下的 mbedtls 精简预设。
cmake_path(CONVERT "$ENV{IDF_PATH}" TO_CMAKE_PATH_LIST ESP_IDF_PATH)
list(APPEND sdkconfig_defaults "${ESP_IDF_PATH}/components/mbedtls/config/mbedtls_preset_bt.conf")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(esp32c3_nus_bridge)
