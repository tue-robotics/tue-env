from typing import Any, List, Mapping, Optional

from os import environ
from functools import partial
import yaml

from lsb_release import get_distro_information

ros_release = environ["TUE_ROS_DISTRO"]
ubuntu_release = get_distro_information()["CODENAME"]


def type_git(install_item: Mapping, allowed_keys: Optional[List[str]] = None) -> Mapping:
    """
    Function to check the parsed yaml for install type git and generate the command string.

    The structure of a git install type is:
    - type: git
      url: <REPOSITORY_URL>
      path: [LOCAL_CLONE_PATH]
      version: [BRANCH_COMMIT_TAG]

    :param install_item: Extracted yaml component corresponding to install type git
    :param allowed_keys: Additional keys to allow apart from the keys defined in install type git
    :return: Mapping string containing repository url and optional keyword arguments target-dir and version
    """

    assert install_item["type"] == "git", f"Invalid install type '{install_item['type']}' for type 'git'"

    keys = {"type", "url", "path", "version"}
    if allowed_keys:
        keys |= set(allowed_keys)

    invalid_keys = [k for k in install_item.keys() if k not in keys]

    if invalid_keys:
        raise KeyError(f"The following keys are invalid for install type 'git': {invalid_keys}")

    url = install_item.get("url")
    target_dir = install_item.get("path")
    version = install_item.get("version")

    if not url:
        raise KeyError("'url' is a mandatory key for install type 'git'")

    return {"url": url, "target_dir": target_dir, "version": version}


def catkin_git(source: Mapping) -> Mapping:
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
    :return: Mapping containing the keyword arguments to the primary catkin target installation command
    """

    args = type_git(source, allowed_keys=["sub-dir"])
    args["source_type"] = "git"

    args["sub_dir"] = source.get("sub-dir")

    return args


def installyaml_parser(installer: Any, path: str, now: bool = False) -> Mapping:
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

    # Combine now calls
    now_cache = {
        "system_now": [],
        "pip_now": [],
        "pip3_now": [],
        "ppa_now": [],
        "snap_now": [],
        "gem_now": [],
    }

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

    for install_item in install_items:
        command = None

        try:
            install_type = install_item["type"]  # type: str
            install_type = install_type.replace("-", "_")  # Need after switching to python impl of tue-get

            if install_type == "empty":
                return {"system_packages": system_packages, "commands": commands}

            elif install_type == "ros":
                try:
                    source = get_distro_item(install_item, "source", ros_release, "ROS")
                except ValueError as e:
                    raise ValueError(f"[{install_type}]: {e.args[0]}")

                # Both release and default are allowed to be None
                if source is None:
                    continue

                source_type = source["type"]
                if source_type == "git":
                    command = partial(getattr(installer, "tue_install_ros"), source_type="git", **catkin_git(source))
                elif source_type == "system":
                    system_packages.append(f"ros-{ros_release}-{source['name']}")
                    command = partial(getattr(installer, "tue_install_ros"), source_type="system", **catkin_git(source))
                else:
                    raise ValueError(f"Unknown ROS install type: '{source_type}'")

            # Non ros targets that are packaged to be built with catkin
            elif install_type == "catkin":
                source = install_item["source"]

                if not source["type"] == "git":
                    raise ValueError(f"Unknown catkin install type: '{source['type']}'")
                command = partial(getattr(installer, "tue_install_ros"), source_type="git", **catkin_git(source))

            elif install_type == "git":
                command = partial(getattr(installer, "tue_install_git"), source_type="git", **type_git(install_item))

            elif install_type in [
                "target",
                "system",
                "pip",
                "pip3",
                "ppa",
                "snap",
                "gem",
                "dpkg",
                "target_now",
                "system_now",
                "pip_now",
                "pip3_now",
                "ppa_now",
                "snap_now",
                "gem_now",
            ]:
                if now and "now" not in install_type:
                    install_type += "_now"

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
                command = partial(getattr(installer, f"tue_install_{install_type}"), pkg_name)

            else:
                raise ValueError(f"Unknown install type: '{install_type}'")

        except KeyError as e:
            raise KeyError(f"Invalid install.yaml file: Key '{e}' could not be found.")

        if not command:
            raise ValueError("Invalid install.yaml file")

        commands.append(command)

    for install_type, pkg_list in now_cache.items():
        if pkg_list:
            command = partial(getattr(installer, f"tue_install_{install_type}"), pkg_list)
            commands.append(command)

    return {"system_packages": system_packages, "commands": commands}


if __name__ == "__main__":
    import sys
    from tue_get.installer_impl import InstallerImpl

    if len(sys.argv) < 2:
        print("Provide yaml file to parse")
        exit(1)

    impl = InstallerImpl(True)
    print(installyaml_parser(impl, sys.argv[1], False))
