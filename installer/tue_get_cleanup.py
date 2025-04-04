#! /usr/bin/env python3

from __future__ import annotations

import sys
from argparse import ArgumentParser
from os import environ
from pathlib import Path

import yaml
from lsb_release import get_distro_information

ubuntu_release = get_distro_information()["CODENAME"]


class Cleanup:
    def __init__(self, tue_env_dir: Path) -> None:
        self.tue_env_dir = tue_env_dir

    @property
    def cleanup_file(self) -> Path:
        return self.tue_env_dir / "cleanup.yaml"

    def _read_cleanup_file(self) -> list[str]:
        with self.cleanup_file.open() as f:
            try:
                cleanup_items = yaml.load(f, yaml.CSafeLoader)
            except AttributeError:
                cleanup_items = yaml.load(f, yaml.SafeLoader)
            except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
                raise ValueError(f"Invalid yaml syntax: {e}")

        if not isinstance(cleanup_items, list):
            raise ValueError("Root of cleanup.yaml file should be a YAML sequence")

        return cleanup_items

    def _write_cleanup_file(self, cleanup_items: list[str]) -> None:
        with self.cleanup_file.open("w") as f:
            yaml.dump(cleanup_items, f)

    def reset_cleanup(self) -> int:
        if self.cleanup_file.exists():
            self.cleanup_file.unlink()

        return 0

    def list_cleanup(self) -> int:
        if not self.cleanup_file.exists():
            return show_error("No cleanup targets defined")

        cleanup_items = self._read_cleanup_file()

        if not cleanup_items:
            print("No cleanup targets defined")
            return 0

        msg = "\n".join(["Cleanup targets:", *cleanup_items])
        print(msg)

        return 0

    def add_target(self, target: str) -> int:
        return self.add_targets([target])

    def add_targets(self, targets: list[str]) -> int:
        if not self.cleanup_file.exists():
            cleanup_items = []
        else:
            cleanup_items = self._read_cleanup_file()

        for target in targets:
            if target not in cleanup_items:
                cleanup_items.append(target)

        self._write_cleanup_file(cleanup_items)

        return 0

    def drop_target(self, target: str) -> int:
        return self.drop_targets([target])

    def drop_targets(self, targets: list[str]) -> int:
        if not self.cleanup_file.exists():
            return show_error("No cleanup targets defined")

        cleanup_items = self._read_cleanup_file()

        for target in targets:
            try:
                cleanup_items.remove(target)
            except ValueError:
                return show_error(f"Target '{target}' not found in cleanup targets")

        self._write_cleanup_file(cleanup_items)

        return 0

    def generate_cleanup_script(self) -> int | str:
        if not self.cleanup_file.exists():
            return show_error("No cleanup targets defined")

        cleanup_items = self._read_cleanup_file()

        if not cleanup_items:
            return show_error("No cleanup targets defined")

        for target in cleanup_items:
            print(f"tue-cleanup {target}")

        return 0




def show_error(error: str) -> int:
    print(f"ERROR: {error}")
    return 1


def type_git(install_item: dict, allowed_keys: list[str] | None = None) -> str:
    """
    Function to check the parsed YAML for install type git and generate the command string.

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

    command: list[str] = [url]

    if path:
        command.append(f"--target-dir={path}")

    if version:
        command.append(f"--version={version}")

    return " ".join(command)


def type_apt_key_source(install_item: dict) -> str:
    dist_exts = install_item.get("distribution-extensions")
    key_file = install_item.get("key-file")
    key_fingerprint = install_item.get("key-fingerprint")
    key_url = install_item.get("key-url")
    repo_components = install_item.get("repo-components", [])  # Optional field
    repo_url = install_item.get("repo-url")
    source_file = install_item.get("source-file")

    args: list[str] = []

    if not isinstance(dist_exts, list):
        raise ValueError("distribution-extensions should be a list")
    if len(dist_exts) < 1:
        raise ValueError("At least one distribution extension should be specified")
    for dist_ext in dist_exts:
        args.append(f"--distribution-extension={dist_ext}")

    args.append(f"--key-file={key_file}")
    args.append(f"--key-fingerprint={key_fingerprint}")
    args.append(f"--key-url={key_url}")

    if not isinstance(repo_components, list):
        raise ValueError("repo-components should be a list")
    for repo_component in repo_components:
        args.append(f"--repo-component={repo_component}")

    args.append(f"--repo-url={repo_url}")
    args.append(f"--source-file={source_file}")

    return " ".join(args)


def catkin_git(source: dict) -> str:
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

    :param source: Extracted YAML component for the key 'source' corresponding to install type catkin/ros
    :return: Command string containing arguments to the primary catkin target installation command
    """

    git_cmd_args = type_git(source, allowed_keys=["sub-dir"])
    command = f"git {git_cmd_args}"

    sub_dir = source.get("sub-dir")
    if sub_dir:
        command += f" --sub-dir={sub_dir}"

    return command


def install_yaml_parser(path: str, now: bool = False) -> dict[str, str]:
    with open(path) as f:
        try:
            install_items = yaml.load(f, yaml.CSafeLoader)
        except AttributeError:
            install_items = yaml.load(f, yaml.SafeLoader)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            raise ValueError(f"Invalid yaml syntax: {e}")

    if not isinstance(install_items, list):
        raise ValueError("Root of install.yaml file should be a YAML sequence")

    system_packages: list[str] = []

    commands: list[str] = []

    def commands_append(command: str) -> None:
        commands.append(command.replace(" ", "^"))

    def get_distro_item(
        item: dict, key: str, release_version: str, release_type: str
    ) -> dict[str, str] | str | None:
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
        "remove-system-now": [],
        "remove-pip-now": [],
        "remove-pip3-now": [],
        "remove-ppa-now": [],
        "remove-snap-now": [],
        "remove-gem-now": [],
    }

    for install_item in install_items:
        command = None

        try:
            install_type = install_item["type"]

            if install_type == "empty":
                return {"system_packages": system_packages, "commands": " ".join(commands)}

            elif install_type == "ros" or install_type == "ros-remove-source":
                # TODO(anyone): Remove the use of TUE_XXX, when migration to TUE_ENV_XXX is complete
                try:
                    ros_release = environ["TUE_ENV_ROS_DISTRO"]
                except KeyError as e:
                    try:
                        ros_release = environ["TUE_ROS_DISTRO"]
                    except KeyError:
                        raise KeyError("TUE_ENV_ROS_DISTRO and TUE_ROS_DISTRO not set in environment variables")

                try:
                    source: dict[str, str] | None = get_distro_item(install_item, "source", ros_release, "ROS")
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
                if now and "now" not in install_type:
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
                if now and "now" not in install_type:
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
    parser = ArgumentParser(description="Parse install.yaml files for cleanup")
    parser.add_argument("--add", "-a", type=str, action="append", help="Add a target to cleanup")
    parser.add_argument("--drop", "-d", type=str, action="append", help="Drop a target from cleanup")
    parser.add_argument("--generate", "-g", action="store_true", help="Generate the cleanup script")
    parser.add_argument("--list", "-l", action="store_true", help="List the cleanup targets")
    parser.add_argument("--reset", "-r", action="store_true", help="Reset the cleanup targets")

    args = parser.parse_args()

    cleanup = Cleanup(Path(environ["TUE_ENV_DIR"]))

    if args.reset:
        return cleanup.reset_cleanup()

    if args.list:
        return cleanup.list_cleanup()

    add_success = cleanup.add_targets(args.add)

    drop_success = cleanup.add_targets(args.drop)

    if not add_success or not drop_success:
        return 1

    if args.generate:
        return cleanup.generate_cleanup_target()

    return 0


if __name__ == "__main__":
    sys.exit(main())
