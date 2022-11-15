#! /usr/bin/env python3

import platform
import sys
import os
from typing import Mapping
from parse_package_xml import packagexml_parser
from parse_install_yaml import installyaml_parser


TUE_ENV_TARGETS_DIR = os.environ["TUE_ENV_TARGETS_DIR"]
TUE_ROS_DISTRO = os.environ["TUE_ROS_DISTRO"]
ARCHITECTURES = {"x86_64": "amd64", "aarch64": "arm64"}
CONTROL_FILE_TEMPLATE = """
Package: {package}
Version: {version}
Architecture: {arch}
Maintainer: {maintainer}
Description: {description}
"""


def main() -> int:
    if len(sys.argv) != 4:
        print("Usage: generate_deb_control RELEASE_DIR PACKAGE.XML TIMESTAMP")
        return 1

    # print(generate_control_file(sys.argv[2])["control"])
    create_dirs(sys.argv[1], sys.argv[2], sys.argv[3])
    return 0


def generate_control_file(path: str) -> Mapping:
    parsed = packagexml_parser(path)["parser"]

    maintainer_string = ", ".join(f"{m} <{parsed['emails'][m]}>" for m in parsed["maintainer"])

    system_deps = []
    for dep in parsed["depend"]:
        install_yaml = os.path.join(TUE_ENV_TARGETS_DIR, dep, "install.yaml")
        if not os.path.exists(install_yaml):
            install_yaml = os.path.join(TUE_ENV_TARGETS_DIR, f"ros-{dep}", "install.yaml")
        system_deps += installyaml_parser(install_yaml)["system_packages"]

    system_dep_string = ""
    if system_deps:
        system_dep_string = ", ".join(set(system_deps))

    version = list(parsed["version"])[0]
    description = list(parsed["description"])[0]
    arch = ARCHITECTURES[platform.machine()]

    build_type = list(parsed["build_type"])

    package = list(parsed["name"])[0]
    package = package.replace("_", "-")
    if "ament_cmake" in build_type:
        package = f"ros-{TUE_ROS_DISTRO}-{package}"

    control_file = CONTROL_FILE_TEMPLATE.format(
        package=package,
        version=version,
        arch=arch,
        maintainer=maintainer_string,
        description=description,
    )
    if system_dep_string:
        control_file += f"Depends: {system_dep_string}\n"

    return {"package": package, "version": version, "arch": arch, "control": control_file}


def create_dirs(release_dir_path: str, package_xml_path: str, timestamp: str) -> None:
    cfdict = generate_control_file(package_xml_path)

    # timestamp = str(datetime.datetime.now()).replace("-", "")
    # timestamp = timestamp.replace(":", "")
    # timestamp = timestamp.replace(".", "")
    # timestamp = timestamp.replace(" ", "")

    rev = f"build{timestamp}"

    package_release_dirname = f"{cfdict['package']}_{cfdict['version']}-{rev}_{cfdict['arch']}"

    control_file_dir = os.path.join(release_dir_path, package_release_dirname, "DEBIAN")
    control_file_path = os.path.join(control_file_dir, "control")

    os.makedirs(control_file_dir)
    with open(control_file_path, "w") as f:
        f.write(cfdict["control"])

    print(package_release_dirname)


if __name__ == "__main__":
    sys.exit(main())
