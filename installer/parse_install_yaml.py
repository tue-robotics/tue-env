#! /usr/bin/env python3

from typing import List, Mapping, Optional
from os import environ
import sys
import yaml

from lsb_release import get_distro_information

ubuntu_release = get_distro_information()["CODENAME"]


def show_error(error: str) -> int:
    print(f"ERROR: {error}")
    return 1


def type_git(install_item: Mapping, allowed_keys: Optional[List[str]] = None) -> str:
    """
    Function to check the parsed yaml for install type git and generate the command string.

    The structure of a git install type is:
    - type: git
      url: <REPOSITORY_URL>
      path: [LOCAL_CLONE_PATH]
      version: [BRANCH_COMMIT_TAG]

    :param install_item: Extracted yaml component corresponding to install type git
    :param allowed_keys: Additional keys to allow apart from the keys defined in install type git
    :return: Command string containing repository url and optional arguments target-dir and version
    """

    assert install_item["type"] == "git", f"Invalid install type '{install_item['type']}' for type 'git'"

    keys = {"type", "url", "path", "version"}
    if allowed_keys:
        keys |= set(allowed_keys)

    invalid_keys = [k for k in install_item.keys() if k not in keys]

    if invalid_keys:
        raise KeyError(f"The following keys are invalid for install type 'git': {invalid_keys}")

    url = install_item.get("url")
    path = install_item.get("path")
    version = install_item.get("version")

    if not url:
        raise KeyError("'url' is a mandatory key for install type 'git'")

    command = url

    if path:
        command += f" --target-dir={path}"

    if version:
        command += f" --version={version}"

    return command


def catkin_git(source: Mapping) -> str:
    """
    Function to generate installation command for catkin git targets from the extracted yaml

    The structure of a catkin git install type is:
    - type: catkin/ros
      source:
        type: git
        url: <REPOSITORY_URL>
        path: [LOCAL_CLONE_PATH]
        version: [BRANCH_COMMIT_TAG]
        sub-dir: [PACKAGE_SUB_DIRECTORY]

    :param source: Extracted yaml component for the key 'source' corresponding to install type catkin/ros
    :return: Command string containing arguments to the primary catkin target installation command
    """

    git_cmd_args = type_git(source, allowed_keys=["sub-dir"])
    command = f"git {git_cmd_args}"

    sub_dir = source.get("sub-dir")
    if sub_dir:
        command += f" --sub-dir={sub_dir}"

    return command


def main() -> int:
    if not 2 <= len(sys.argv) <= 3:
        return show_error("Usage: parse_install_yaml install.yaml [--now]")

    now = False
    if len(sys.argv) == 3:
        if sys.argv[2] == "--now":
            now = True
        else:
            return show_error(f"Unknown option: {sys.argv[2]}")

    try:
        result = installyaml_parser(sys.argv[1], now)
    except Exception as e:
        return show_error(str(e))

    if result["commands"]:
        print(result["commands"])

    return 0


def installyaml_parser(path: str, now: bool = False) -> Mapping:
    with open(path) as f:
        try:
            install_items = yaml.load(f, yaml.CSafeLoader)
        except AttributeError:
            install_items = yaml.load(f, yaml.SafeLoader)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            raise ValueError(f"Invalid yaml syntax: {e}")

    if not isinstance(install_items, list):
        raise ValueError("Root of install.yaml file should be a YAML sequence")

    system_packages = []

    commands = []

    def commands_append(command: str) -> None:
        command = command.replace(" ", "^")
        commands.append(command)

    def get_distro_item(item: Mapping, key: str, release_version: str, release_type: str) -> Optional[str]:
        if key in item:
            value = item[key]
            if value is None:
                raise ValueError(f"'{key}' is defined, but has no value")
        elif len(item) < 3 or "default" not in item:
            raise ValueError(f"At least one distro and 'default' should be specified or none in install.yaml")
        else:
            for version in [release_version, "default"]:
                if version in item:
                    value = item[version][key]
                    break

        return value

    # Combine now calls
    now_cache = {
        "system-now": [],
        "pip-now": [],
        "pip3-now": [],
        "ppa-now": [],
        "snap-now": [],
        "gem-now": [],
    }

    for install_item in install_items:
        command = None

        try:
            install_type = install_item["type"]

            if install_type == "empty":
                return {"system_packages": system_packages, "commands": commands}

            elif install_type == "ros":
                ros_release = environ["TUE_ROS_DISTRO"]
                try:
                    source = get_distro_item(install_item, "source", ros_release, "ROS")
                except ValueError as e:
                    raise ValueError(f"[{install_type}]: {e.args[0]}")

                # Both release and default are allowed to be None
                if source is None:
                    continue

                source_type = source["type"]
                if source_type == "git":
                    command = f"tue-install-ros {catkin_git(source)}"
                elif source_type == "system":
                    system_packages.append(f"ros-{ros_release}-{source['name']}")
                    command = f"tue-install-ros system {source['name']}"
                else:
                    raise ValueError(f"Unknown ROS install type: '{source_type}'")

            # Non ros targets that are packaged to be built with catkin
            elif install_type == "catkin":
                source = install_item["source"]

                if not source["type"] == "git":
                    raise ValueError(f"Unknown catkin install type: '{source['type']}'")
                command = f"tue-install-ros {catkin_git(source)}"

            elif install_type == "git":
                command = f"tue-install-git {type_git(install_item)}"

            elif install_type in [
                "target",
                "system",
                "pip",
                "pip3",
                "ppa",
                "snap",
                "gem",
                "dpkg",
                "target-now",
                "system-now",
                "pip-now",
                "pip3-now",
                "ppa-now",
                "snap-now",
                "gem-now",
            ]:
                if now and "now" not in install_type:
                    install_type += "-now"

                try:
                    pkg_name = get_distro_item(install_item, "name", ubuntu_release, "Ubuntu")
                except ValueError as e:
                    raise ValueError(f"[{install_type}]: {e.args[0]}")

                # Both release and default are allowed to be None
                if pkg_name is None:
                    continue

                if "system" in install_type:
                    system_packages.append(pkg_name)

                # Cache install types which accept multiple pkgs at once
                if install_type in now_cache:
                    now_cache[install_type].append(pkg_name)
                    continue

                command = f"tue-install-{install_type} {pkg_name}"

            else:
                raise ValueError(f"Unknown install type: '{install_type}'")

        except KeyError as e:
            raise KeyError(f"Invalid install.yaml file: Key '{e}' could not be found.")

        if not command:
            raise ValueError("Invalid install.yaml file")

        commands_append(command)

    for install_type, pkg_list in now_cache.items():
        if pkg_list:
            pkg_list = " ".join(pkg_list)
            command = f"tue-install-{install_type} {pkg_list}"
            commands_append(command)

    commands = " ".join(commands)

    return {"system_packages": system_packages, "commands": commands}


if __name__ == "__main__":
    sys.exit(main())
