#!/usr/bin/env python3

import argparse
import os
import platform
import shutil
import subprocess
import urllib.request
import zipfile
from pathlib import Path

SYSTEM = platform.system()
IS_WINDOWS = SYSTEM == "Windows"
IS_LINUX = SYSTEM == "Linux"
IS_OSX = SYSTEM == "Darwin"

assert IS_WINDOWS or IS_LINUX or IS_OSX, "Unsupported platform"

JOLTC_CONFIG = "Release"
JOLTC_PATH = "joltc"
BINDGEN_PATH = "odin-c-bindgen"

JOLTC_ZIP_URL = "https://github.com/amerkoleci/joltc/archive/refs/heads/main.zip"
BINDGEN_ZIP_URL = (
    "https://github.com/karl-zylinski/odin-c-bindgen/archive/refs/heads/main.zip"
)


def main():
    parser = argparse.ArgumentParser(
        prog="build.py",
        description="Cross-platform build script for joltc and odin-c-bindgen",
    )
    parser.add_argument(
        "-build-joltc",
        action="store_true",
        help="Compile joltc",
    )
    parser.add_argument(
        "-update-joltc",
        action="store_true",
        help="Update joltc",
    )
    parser.add_argument(
        "-build-bindgen",
        action="store_true",
        help="Compile odin-c-bindgen",
    )
    parser.add_argument(
        "-update-bindgen",
        action="store_true",
        help="Update odin-c-bindgen",
    )
    parser.add_argument(
        "-gen-bindings", action="store_true", help="Generate odin bindings for joltc"
    )

    args = parser.parse_args()

    if not any(vars(args).values()):
        parser.print_help()
        return

    # joltc commands

    do_update_joltc = args.update_joltc

    if not os.path.exists(JOLTC_PATH) or len(os.listdir(JOLTC_PATH)) == 0:
        do_update_joltc = True

    do_build_joltc = args.build_joltc or do_update_joltc

    if do_build_joltc:
        build_joltc(do_update_joltc)

    # Bindgen commands

    do_gen_bindings = args.gen_bindings

    do_update_bindgen = args.update_bindgen

    if not os.path.exists(BINDGEN_PATH) or len(os.listdir(BINDGEN_PATH)) == 0:
        do_update_bindgen = True

    do_build_bindgen = args.build_bindgen or do_update_bindgen

    if (
        do_update_bindgen
        or do_gen_bindings
        and not any(
            f.stem == "bindgen" for f in Path("odin-c-bindgen").iterdir() if f.is_file()
        )
    ):
        do_build_bindgen = True

    if do_build_bindgen:
        build_bindgen(do_update_bindgen)

    if do_gen_bindings:
        gen_bindings()

    print("Done!")


def run(cmd, cwd=None):
    print(f"-> Running: {' '.join(cmd)}")
    subprocess.run(cmd, cwd=cwd, check=True)


def build_joltc(update: bool):
    if update:
        print("Updating Joltc...")

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

    print("Compile joltc...")
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
        shutil.copy(build_dir / "lib" / "libjoltc.dylib", Path("jolt"))


def build_bindgen(update: bool):
    if update:
        print("Updating odin-c-bindgen...")

        if os.path.exists(BINDGEN_PATH):
            shutil.rmtree(BINDGEN_PATH)

        temp_zip = "bindgen-temp.zip"
        temp_folder = "bindgen-temp"
        print("Downloading odin-c-bindgen...")
        urllib.request.urlretrieve(BINDGEN_ZIP_URL, temp_zip)

        with zipfile.ZipFile(temp_zip) as zip_file:
            zip_file.extractall(temp_folder)
            shutil.copytree(temp_folder + "/odin-c-bindgen-main", BINDGEN_PATH)

        os.remove(temp_zip)
        shutil.rmtree(temp_folder)

    print("Compile odin-c-bindgen...")

    if IS_WINDOWS:
        run(["odin", "build", "src", "-out:bindgen.exe"], cwd="odin-c-bindgen")
    elif IS_LINUX or IS_OSX:
        run(["odin", "build", "src", "-out:bindgen.bin"], cwd="odin-c-bindgen")


def gen_bindings():
    if IS_WINDOWS:
        run(["odin-c-bindgen/bindgen.exe", "bindgen"])
    if IS_LINUX or IS_OSX:
        run(["odin-c-bindgen/bindgen.bin", "bindgen"])


main()
