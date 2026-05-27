#!/usr/bin/env python3
"""Idempotently add FoundBlock + FoundBlockArray to an existing cc_interfaces.

Preserves the existing maintainer, license, description, and any other
fields. Safe to run on every deploy: each insertion is gated on a
substring check that becomes true after the first run.

Usage:
    python3 apply_patch.py <path-to-cc_interfaces>
"""

import os
import shutil
import sys


def main(cc_path):
    here = os.path.dirname(os.path.abspath(__file__))
    msg_dir = os.path.join(cc_path, "msg")
    os.makedirs(msg_dir, exist_ok=True)
    for name in ("FoundBlock.msg", "FoundBlockArray.msg"):
        shutil.copy(os.path.join(here, "msg", name), msg_dir)

    cmake_path = os.path.join(cc_path, "CMakeLists.txt")
    with open(cmake_path) as f:
        cmake = f.read()

    if "find_package(std_msgs" not in cmake:
        cmake = cmake.replace(
            "find_package(action_msgs REQUIRED)",
            "find_package(action_msgs REQUIRED)\nfind_package(std_msgs REQUIRED)",
        )

    if '"msg/FoundBlock.msg"' not in cmake:
        cmake = cmake.replace(
            '"action/ManipulationTask.action"',
            '"action/ManipulationTask.action"\n'
            '  "msg/FoundBlock.msg"\n'
            '  "msg/FoundBlockArray.msg"',
        )

    if "DEPENDENCIES geometry_msgs action_msgs std_msgs" not in cmake:
        cmake = cmake.replace(
            "DEPENDENCIES geometry_msgs action_msgs",
            "DEPENDENCIES geometry_msgs action_msgs std_msgs",
        )

    with open(cmake_path, "w") as f:
        f.write(cmake)

    pkg_path = os.path.join(cc_path, "package.xml")
    with open(pkg_path) as f:
        pkg = f.read()

    if "<depend>std_msgs</depend>" not in pkg:
        pkg = pkg.replace(
            "<depend>action_msgs</depend>",
            "<depend>action_msgs</depend>\n  <depend>std_msgs</depend>",
        )

    with open(pkg_path, "w") as f:
        f.write(pkg)

    print("cc_interfaces patched")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "cc_interfaces")
