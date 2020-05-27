import os
import glob
Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#


def sdk_prepare():
    try:
        import configparser
    except ImportError:
        import ConfigParser as configparser

    config = configparser.ConfigParser()
    config.read("platformio.ini")

    sdk_root = config.get("extra", "sdk_root")

    src_to_exclude  = glob.glob(sdk_root + '/modules/nrfx/mdk/arm*.s')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/iar*.s')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/ses*.s')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/gcc_startup_nrf5?.S')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/gcc_startup_nrf9*.S')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/gcc_startup_nrf528[!4]*.S')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/system_nrf5?.c')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/system_nrf9*.c')
    src_to_exclude += glob.glob(sdk_root + '/modules/nrfx/mdk/system_nrf528[!4]*.c')
    src_to_exclude += glob.glob(sdk_root + '/components/ble/ble_services/ble_escs/nrf_ble_escs.c')
    src_to_exclude += glob.glob(sdk_root + '/components/ble/ble_services/eddystone/*.[ch]')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/bootloader/ant_dfu/*.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/bootloader/dfu/nrf_dfu_trigger_usb.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/bootloader/serial_dfu/*.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/crypto/nrf_crypto_svc.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/experimental_task_manager/*armgcc*.S')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/experimental_task_manager/*iar*.s')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/experimental_task_manager/*keil*.s')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/scheduler/app_scheduler_serconn.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/timer/app_timer_[fr]*.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/timer/app_timer.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/util/*iar*.c')
    src_to_exclude += glob.glob(sdk_root + '/components/libraries/util/*keil*.c')
    src_to_exclude += glob.glob(sdk_root + '/external/freertos/portable/CMSIS/nrf51/*.[ch]')

    for i in src_to_exclude:
        os.replace(i, i + '.none')


sdk_prepare()

env.Append(
    LINKFLAGS=[
        "-mthumb",
        "-mabi=aapcs",
        "-mcpu=cortex-m4",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-lc",
        "-lnosys",
        "-lm",
        "-Wl,--gc-sections"
    ]
)