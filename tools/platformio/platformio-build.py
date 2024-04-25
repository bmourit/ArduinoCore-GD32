# Copyright 2014-present PlatformIO <contact@platformio.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Extended and rewritten by Maximilian Gerhardt <maximilian.gerhardt@rub.de>
# for GD32 core.

"""
Arduino

Arduino Wiring-based Framework allows writing cross-platform software to
control devices attached to a wide range of Arduino boards to create all
kinds of creative coding, interactive objects, spaces or physical experiences.

https://github.com/CommunityGD32Cores/ArduinoCore-GD32
"""

from os.path import isfile, isdir, join

from SCons.Script import COMMAND_LINE_TARGETS, DefaultEnvironment

# get environment
env = DefaultEnvironment()
platform = env.PioPlatform()
board_config = env.BoardConfig()

# check framework is installed
FRAMEWORK_DIR = platform.get_package_dir("framework-arduinogd32")
#CMSIS_DIR = join(platform.get_package_dir("framework-arduinogd32"), "CMSIS", "CMSIS")
assert isdir(FRAMEWORK_DIR)
#assert isdir(CMSIS_DIR)

VARIANT_REMAP = {
    "CREALITY_422_GD32F303RE": "CREALITY_422_GD32F303RE"
}

def get_variant(board):
    variant = VARIANT_REMAP[board] if board in VARIANT_REMAP else board_config.get("build.variant")
    return variant

# get mcu and board variant
mcu = board_config.get("build.mcu", "")
board_name = env.subst("$BOARD")
mcu_type = mcu[:-2]
variant = get_variant(board_name) #board_config.get("build.variant")
series = mcu_type[:7].upper() + "x"
spl_series = board_config.get("build.spl_series", "")
variants_dir = (
    join("$PROJECT_DIR", board_config.get("build.variants_dir"))
    if board_config.get("build.variants_dir", "")
    else join(FRAMEWORK_DIR, "variants")
)
variant_dir = join(variants_dir, variant)
upload_protocol = env.subst("$UPLOAD_PROTOCOL")

def process_standard_library_configuration(cpp_defines):
    if "PIO_FRAMEWORK_ARDUINO_STANDARD_LIB" in cpp_defines:
        env["LINKFLAGS"].remove("--specs=nano.specs")
    if "PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_PRINTF" in cpp_defines:
        env.Append(LINKFLAGS=["-u_printf_float"])
    if "PIO_FRAMEWORK_ARDUINO_NANOLIB_FLOAT_SCANF" in cpp_defines:
        env.Append(LINKFLAGS=["-u_scanf_float"])

def process_usart_configuration(cpp_defines):
    if "PIO_FRAMEWORK_ARDUINO_SERIAL_DISABLED" in cpp_defines:
        env["CPPDEFINES"].remove("HAL_UART_MODULE_ENABLED")

    elif "PIO_FRAMEWORK_ARDUINO_SERIAL_WITHOUT_GENERIC" in cpp_defines:
        env.Append(CPPDEFINES=["HWSERIAL_NONE"])

def add_upload_protocol_defines(upload_protocol):
    if upload_protocol == "serial":
        env.Append(CPPDEFINES=[("CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG", 1)])
    elif upload_protocol == "dfu":
        env.Append(CPPDEFINES=["SERIAL_USB"])
    else:
        env.Append(CPPDEFINES=[("CONFIG_MAPLE_MINI_NO_DISABLE_DEBUG", 1), "SERIAL_USB"])

    is_generic = 0

    if "generic" in board_name.lower():
        is_generic = 1

    if upload_protocol in ("stlink", "dfu", "jlink") and is_generic:
        env.Append(CPPDEFINES=["GENERIC_BOOTLOADER"])

def get_arm_math_lib(cpu):
    core = board_config.get("build.cpu")
    if "m4" in core:
        return "arm_cortexM4lf_math"
    elif "m7" in core:
        return "arm_cortexM7lfsp_math"
    elif "m33" in core:
        return "arm_ARMv8MMLlfsp_math"

    return "arm_cortex%sl_math" % core[7:9].upper()

def configure_application_offset(mcu, upload_protocol):
    offset = 0

    if upload_protocol == "hid":
        if mcu.startswith("gd32f1"):
            offset = 0x800
        elif mcu.startswith("gd32f3"):
            offset = 0x800
        elif mcu.startswith("gd32f4"):
            offset = 0x4000

        env.Append(CPPDEFINES=["BL_HID"])

    elif upload_protocol == "dfu":
        # GD32F103/GD32F303 series don't have embedded DFU over USB
        # stm32duino bootloader (v1, v2) is used instead
        if mcu.startswith("gd32f303"):
            if board_config.get("upload.boot_version", 2) == 1:
                offset = 0x5000
            else:
                offset = 0x2000
            env.Append(CPPDEFINES=["BL_LEGACY_LEAF"])

    if offset != 0:
        env.Append(CPPDEFINES=[("VECT_TAB_OFFSET", "%s" % hex(offset))],)

    # LD_FLASH_OFFSET is mandatory even if there is no offset
    env.Append(LINKFLAGS=["-Wl,--defsym=LD_FLASH_OFFSET=%s" % hex(offset)])

machine_flags = [
    "-mcpu=%s" % board_config.get("build.cpu"),
    "-mthumb",
]

#if any(mcu in board_config.get("build.cpu") for cpu in ("cortex-m4", "cortex-m7")):
#    machine_flags.extend(["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"])

if board_config.get("build.cpu") == "cortex-m33":
    machine_flags.extend(["-mfpu=fp-armv8", "-mfloat-abi=softfp"])

env.Append(
    ASFLAGS=machine_flags
    + ["-x", "assembler-with-cpp"],
    ASPPFLAGS=["-x", "assembler-with-cpp"],
    CFLAGS=["-std=gnu17"],
    CXXFLAGS=[
        "-std=gnu++17",
        "-fno-threadsafe-statics",
        "-fno-rtti",
        "-fno-exceptions",
        "-fno-use-cxa-atexit",
    ],
    CCFLAGS=machine_flags
    + [
        "-Os",  # optimize for size
        "-ffunction-sections",  # place each function in its own section
        "-fdata-sections",
        "-Wall",
        "-nostdlib",
        "--param",
        "max-inline-insns-single=500",
    ],
    CPPDEFINES=[
        series,
        spl_series,
        ("ARDUINO", 10808),
        ("F_CPU", "$BOARD_F_CPU"), # for compatiblity
        "ARDUINO_ARCH_GD32",
        "ARDUINO_%s" % board_name.upper(),
        ("BOARD_NAME", '\\"%s\\"' % board_name.upper()),
        ("ARDUINO_UPLOAD_MAXIMUM_SIZE", board_config.get("upload.maximum_size")),
    ],
    CPPPATH=[
        join(FRAMEWORK_DIR, "cores", "arduino", "api", "deprecated"),
        join(FRAMEWORK_DIR, "cores", "arduino", "api", "deprecated-avr-comp"),
        join(FRAMEWORK_DIR, "cores", "arduino", "gd32"),
        join(FRAMEWORK_DIR, "cores", "arduino", "gd32", "Source"),
        join(FRAMEWORK_DIR, "system", "startup"),
        join(FRAMEWORK_DIR, "system", "CMSIS", "Core", "Include"),
        join(FRAMEWORK_DIR, "system", "CMSIS", "DSP", "Include"),
        join(FRAMEWORK_DIR, "system", spl_series + "_firmware", "CMSIS"),
        join(FRAMEWORK_DIR, "system", spl_series + "_firmware", "CMSIS", "GD", spl_series, "Include"),
        join(FRAMEWORK_DIR, "system", spl_series + "_firmware", "CMSIS", "GD", spl_series, "Source"),
        join(FRAMEWORK_DIR, "system", spl_series + "_firmware", spl_series + "_standard_peripheral", "Include"),
        join(FRAMEWORK_DIR, "system", spl_series + "_firmware", spl_series + "_standard_peripheral", "Source"),
        join(FRAMEWORK_DIR, "cores", "arduino"),
        variant_dir,
    ],
    LINKFLAGS=machine_flags
    + [
        "-Os",
        "--specs=nano.specs",
        "-Wl,--gc-sections,--relax",
        "-Wl,--check-sections",
        "-Wl,--entry=Reset_Handler",
        "-Wl,--unresolved-symbols=report-all",
        "-Wl,--warn-common",
        "-Wl,--defsym=LD_MAX_SIZE=%d" % board_config.get("upload.maximum_size"),
        "-Wl,--defsym=LD_MAX_DATA_SIZE=%d" % board_config.get("upload.maximum_ram_size"),
    ],
    LIBS=[
        get_arm_math_lib(env.BoardConfig().get("build.cpu")),
        "c",
        "m",
        "gcc",
        "stdc++",
    ],
    LIBPATH=[variant_dir, join(FRAMEWORK_DIR, "system", "CMSIS", "DSP", "Lib", "GCC")],
)

env.ProcessFlags(board_config.get("build.framework_extra_flags.arduino", ""))

configure_application_offset(mcu, upload_protocol)

#
# Linker requires preprocessing with correct RAM|ROM sizes
#

if not board_config.get("build.ldscript", ""):
    if not isfile(join(env.subst(variant_dir), "ldscript.ld")):
        print("Warning! Cannot find linker script for the current target!\n")
    env.Replace(LDSCRIPT_PATH=join(variant_dir, "ldscript.ld"))

#
# Process configuration flags
#
cpp_defines = env.Flatten(env.get("CPPDEFINES", []))

process_standard_library_configuration(cpp_defines)
add_upload_protocol_defines(upload_protocol)
#process_usb_configuration(cpp_defines)
#process_usart_configuration(cpp_defines)

# copy CCFLAGS to ASFLAGS (-x assembler-with-cpp mode)
env.Append(ASFLAGS=env.get("CCFLAGS", [])[:])

env.Append(
    LIBSOURCE_DIRS=[
        join(FRAMEWORK_DIR, "libraries", "__cores__" "arduino"),
        join(FRAMEWORK_DIR, "libraries"),
    ]
)

#
# Target: Build Core Library
#
libs = []

if "build.variant" in board_config:
    env.Append(CPPPATH=[variant_dir])
    env.BuildSources(join("$BUILD_DIR", "FrameworkArduinoVariant"), variant_dir)

env.BuildSources(
    join("$BUILD_DIR", "FrameworkArduino"), join(FRAMEWORK_DIR, "cores", "arduino")
)

env.Prepend(LIBS=libs)
