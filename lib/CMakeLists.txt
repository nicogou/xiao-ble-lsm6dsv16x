# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_CUSTOM custom)
add_subdirectory_ifdef(CONFIG_XIAO_BLE_SHELL xiao_ble_shell)
add_subdirectory_ifdef(CONFIG_LSM6DSV16X lsm6dsv16x)
add_subdirectory_ifdef(CONFIG_LSM6DSV16BX lsm6dsv16bx)
add_subdirectory_ifdef(CONFIG_XIAO_SMP_BLUETOOTH smp_bluetooth)
add_subdirectory_ifdef(CONFIG_FIT_SDK fit_sdk)
