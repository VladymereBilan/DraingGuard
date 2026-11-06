"""Post-build step: produce a full flash image for the Wokwi simulator.

Wokwi loads a plain application .bin (PlatformIO's firmware.bin) by supplying its
own bootloader and a *default* partition table - and that table has no data
partition, so LittleFS fails with:

    E esp_littlefs: partition "spiffs" could not be found

The fix is to hand Wokwi a full image containing the bootloader, the partition
table and the app, built with `esptool merge_bin`. wokwi.toml points at the
merged file; the real esp32 environment is untouched and still flashes the
ordinary app image over serial.

Wired up from platformio.ini as `extra_scripts = post:merge_image.py` on the
wokwi env only.
"""

import os

Import("env")  # noqa: F821  (injected by SCons)


MERGED_NAME = "firmware.merged.bin"


def merge_full_image(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    app_bin = os.path.join(build_dir, env.subst("$PROGNAME") + ".bin")
    merged = os.path.join(build_dir, MERGED_NAME)

    # FLASH_EXTRA_IMAGES carries the (offset, path) pairs the platform would
    # flash alongside the app - bootloader at 0x1000, partitions at 0x8000 and
    # boot_app0 at 0xe000 - so the offsets never have to be hard-coded here.
    segments = []
    for offset, image in env.get("FLASH_EXTRA_IMAGES", []):
        segments += [str(offset), env.subst(image)]
    segments += [env.subst("$ESP32_APP_OFFSET"), app_bin]

    board = env.BoardConfig()
    flash_mode = board.get("build.flash_mode", "dio")
    flash_size = board.get("upload.flash_size", "4MB")
    flash_freq = board.get("build.f_flash", "40000000L")
    flash_freq = str(flash_freq).replace("000000L", "m").replace("000000", "m")

    cmd = [
        '"$PYTHONEXE"',
        '"$OBJCOPY"',
        "--chip",
        "esp32",
        "merge_bin",
        "-o",
        '"%s"' % merged,
        "--flash_mode",
        flash_mode,
        "--flash_freq",
        flash_freq,
        "--flash_size",
        flash_size,
    ]
    for item in segments:
        cmd.append(item if item.startswith("0x") else '"%s"' % item)

    env.Execute(
        env.VerboseAction(" ".join(cmd), "Merging full flash image $BUILD_DIR/" + MERGED_NAME)
    )


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge_full_image)  # noqa: F821
