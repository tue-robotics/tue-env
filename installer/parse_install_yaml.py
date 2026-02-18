#! /usr/bin/env python3

import sys
import traceback
from os import environ
from pathlib import Path
from typing import List, Mapping, Optional, Union

import yaml
from lsb_release import get_distro_information

ubuntu_release = get_distro_information()["CODENAME"]


def type_git(install_item: Mapping, allowed_keys: Optional[List[str]] = None) -> str:
    """
    Function to check the parsed YAML for install type git and generate the command string.

    The structure of a git install type is:
    - type: git
      url: <REPOSITORY_URL>
      path: [LOCAL_CLONE_PATH]
      version: [BRANCH_COMMIT_TAG]

    :param install_item: Extracted YAML component corresponding to install type git
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

    command: List[str] = [url]

    if path:
        command.append(f"--target-dir={path}")

    if version:
        command.append(f"--version={version}")

    return " ".join(command)


def type_apt_key_source(install_item: Mapping) -> str:
    dist_exts = install_item.get("distribution-extensions", [])  # Optional field
    include_distribution = install_item.get("include-distribution", True)  # Optional field
    key_file = install_item.get("key-file")
    key_fingerprint = install_item.get("key-fingerprint")
    key_url = install_item.get("key-url")
    repo_components = install_item.get("repo-components", [])  # Optional field
    repo_name = install_item.get("repo-name", "")  # Optional field
    repo_url = install_item.get("repo-url")
    source_file = install_item.get("source-file")

    args: List[str] = []

    args.append(f"--include-distribution={str(include_distribution).lower()}")

    if not isinstance(dist_exts, list):
        raise ValueError("distribution-extensions should be a list")
    for dist_ext in dist_exts:
        args.append(f"--distribution-extension={dist_ext}")

    args.append(f"--key-file={key_file}")
    args.append(f"--key-fingerprint={key_fingerprint}")
    args.append(f"--key-url={key_url}")

    if not isinstance(repo_components, list):
        raise ValueError("repo-components should be a list")
    for repo_component in repo_components:
        args.append(f"--repo-component={repo_component}")

    if repo_name:
        args.append(f"--repo-name={repo_name}")
    args.append(f"--repo-url={repo_url}")
    args.append(f"--source-file={source_file}")

    return " ".join(args)


def catkin_git(source: Mapping) -> str:
    """
    Function to generate installation command for catkin git targets from the extracted YAML

    The structure of a catkin git install type is:
    - type: catkin/ros
      source:
        type: git
        url: <REPOSITORY_URL>
        path: [LOCAL_CLONE_PATH]
        version: [BRANCH_COMMIT_TAG]
        sub-dir: [PACKAGE_SUB_DIRECTORY]

    :param source: Extracted YAML component for the key 'source' corresponding to install type catkin/ros
    :return: Command string containing arguments to the primary catkin target installation command
    """

    git_cmd_args = type_git(source, allowed_keys=["sub-dir"])
    command = f"git {git_cmd_args}"

    sub_dir = source.get("sub-dir")
    if sub_dir:
        command += f" --sub-dir={sub_dir}"

    return command


def install_yaml_parser(path: Path, now: bool = False) -> Mapping[str, str]:
    with path.open() as f:
        try:
            install_items = yaml.load(f, yaml.CSafeLoader)
        except AttributeError:
            install_items = yaml.load(f, yaml.SafeLoader)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            raise ValueError(f"Invalid yaml syntax: {e}")

    if not isinstance(install_items, list):
        raise ValueError("Root of install.yaml file should be a YAML sequence")

    system_packages: List[str] = []

    commands: List[str] = []

    def commands_append(command: str) -> None:
        commands.append(command.replace(" ", "^"))

    def get_distro_item(
        item: Mapping, key: str, release_version: str, release_type: str
    ) -> Optional[Union[Mapping[str, str], str]]:
        if key in item:
            value = item[key]
            if value is None:
                raise ValueError(f"'{key}' is defined, but has no value")
        elif len(item) < 3 or "default" not in item:
            raise ValueError(
                f"At least one {release_type} distro and 'default' should be specified or none in install.yaml"
            )
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
                return {"system_packages": system_packages, "commands": " ".join(commands)}

            elif install_type == "ros" or install_type == "ros-remove-source":
                ros_release = environ["TUE_ENV_ROS_DISTRO"]

                try:
                    source: Optional[Mapping[str, str]] = get_distro_item(install_item, "source", ros_release, "ROS")
                except ValueError as e:
                    raise ValueError(f"[{install_type}]: {e.args[0]}")

                # Both release and default are allowed to be None
                if source is None:
                    continue

                if install_type == "ros":
                    source_type = source["type"]
                    if source_type == "git":
                        command = f"tue-install-ros {catkin_git(source)}"
                    elif source_type == "system":
                        system_packages.append(f"ros-{ros_release}-{source['name']}")
                        command = f"tue-install-ros system {source['name']}"
                    else:
                        raise ValueError(f"Unknown ROS install type: '{source_type}'")
                elif install_type == "ros-remove-source":
                    command_list = ["tue-install-ros-remove-source", type_git(source, ["eol"])]

                    if "eol" not in source:
                        raise KeyError("EOL is a mandatory key for install type 'ros-remove-source'")
                    command_list.append(f"--eol={source['eol']}")

                    command = " ".join(command_list)

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
                if now and not install_type.endswith("-now"):
                    install_type += "-now"

                try:
                    pkg_name: str = get_distro_item(install_item, "name", ubuntu_release, "Ubuntu")
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

            elif install_type in ["apt-key-source", "apt-key-source-now"]:
                if now and not install_type.endswith("-now"):
                    install_type += "-now"

                command = f"tue-install-{install_type} {type_apt_key_source(install_item)}"

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

    return {"system_packages": system_packages, "commands": " ".join(commands)}


def main() -> int:
    if not 2 <= len(sys.argv) <= 3:
        print("Usage: parse_install_yaml install.yaml [--now]")
        return 1

    now: bool = False
    if len(sys.argv) == 3:
        if sys.argv[2] == "--now":
            now = True
        else:
            print(f"Unknown option: {sys.argv[2]}")
            return 1

    try:
        path = Path(sys.argv[1])
        result = install_yaml_parser(path, now)
    except Exception as e:
        print(f"ERROR: Could not parse install.yaml: {repr(e)}\n{traceback.format_exc()}")
        return 1

    if result["commands"]:
        print(result["commands"])

    return 0


if __name__ == "__main__":
    sys.exit(main())
