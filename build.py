#!/usr/bin/env python3

import argparse
import os
import platform
import shutil
import subprocess
import urllib.request
import zipfile
from pathlib import Path

JOLTC_CONFIG = "Release"
JOLTC_PATH = "joltc"

SYSTEM = platform.system()
IS_WINDOWS = SYSTEM == "Windows"
IS_LINUX = SYSTEM == "Linux"
IS_OSX = SYSTEM == "Darwin"

assert IS_WINDOWS or IS_LINUX or IS_OSX, "Unsupported platform"


def main():
    parser = argparse.ArgumentParser(
        prog="build.py",
        description="Cross-platform build script for joltc and odin-c-bindgen",
    )
    parser.add_argument(
        "-update-joltc", action="store_true", help="Download latest joltc library"
    )
    parser.add_argument(
        "-compile-joltc",
        action="store_true",
        help="Compile joltc for current platform",
    )
    args = parser.parse_args()

    if not any(vars(args).values()):
        parser.print_help()
        return

    do_update = args.update_joltc

    if not os.path.exists(JOLTC_PATH) or len(os.listdir(JOLTC_PATH)) == 0:
        do_update = True

    if do_update:
        update_joltc()

    do_compile = do_update or args.compile_joltc

    if do_compile:
        compile_joltc()

    print("Done!")


def run(cmd, cwd=None):
    print(f"→ Running: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=cwd, check=True)


def update_joltc():
    print("Updating Joltc...")
    JOLTC_ZIP_URL = "https://github.com/amerkoleci/joltc/archive/refs/heads/main.zip"

    if os.path.exists(JOLTC_PATH):
        shutil.rmtree(JOLTC_PATH)

    temp_zip = "joltc-temp.zip"
    temp_folder = "joltc-temp"
    print("Downloading joltc...")
    urllib.request.urlretrieve(JOLTC_ZIP_URL, temp_zip)

    with zipfile.ZipFile(temp_zip) as zip_file:
        zip_file.extractall(temp_folder)
        shutil.copytree(temp_folder + "/joltc-main", JOLTC_PATH)

    os.remove(temp_zip)
    shutil.rmtree(temp_folder)


def compile_joltc():
    print("Compile Joltc...")
    build_dir = Path("joltc") / "build"

    run(
        [
            "cmake",
            "-S",
            ".",
            "-B",
            "build",
            "-DJPH_SAMPLES=OFF",
            "-DJPH_BUILD_SHARED=ON",
            "-DCMAKE_BUILD_TYPE=" + JOLTC_CONFIG,
        ],
        cwd="joltc",
    )
    run(["cmake", "--build", "build", "--config", JOLTC_CONFIG], cwd="joltc")

    if IS_WINDOWS:
        shutil.copy(build_dir / "bin" / JOLTC_CONFIG / "joltc.dll", Path.cwd())
        shutil.copy(build_dir / "lib" / JOLTC_CONFIG / "joltc.lib", Path("jolt"))
    elif IS_LINUX:
        shutil.copy(build_dir / "lib" / "libjoltc.so", Path("jolt"))
    elif IS_OSX:
        print("TODO!")


main()
