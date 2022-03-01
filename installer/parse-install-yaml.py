#! /usr/bin/env python3

from os import environ
import sys
import yaml

from lsb_release import get_distro_information

ros_release = environ["TUE_ROS_DISTRO"]
ubuntu_release = get_distro_information()["CODENAME"]


def show_error(error):
    print("ERROR: {0}".format(error))
    return 1


def type_git(install_item: dict, allowed_keys: list = None) -> str:
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

    assert install_item["type"] == "git", "Invalid install type '{}' for type 'git'".format(install_item["type"])

    keys = ["type", "url", "path", "version"]
    if allowed_keys:
        keys += allowed_keys

    invalid_keys = [k for k in install_item.keys() if k not in set(keys)]

    if invalid_keys:
        raise KeyError("The following keys are invalid for install type 'git': {}".format(invalid_keys))

    url = install_item.get("url")
    path = install_item.get("path")
    version = install_item.get("version")

    if not url:
        raise KeyError("'url' is a mandatory key for install type 'git'")

    command = "{}".format(url)

    if path:
        command += " --target-dir={}".format(path)

    if version:
        command += " --version={}".format(version)

    return command


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

    :param source: Extracted yaml component for the key 'source' corresponding to install type catkin/ros
    :return: Command string containing arguments to the primary catkin target installation command
    """

    git_cmd_args = type_git(source, allowed_keys=["sub-dir"])
    command = "git {}".format(git_cmd_args)

    sub_dir = source.get("sub-dir")
    if sub_dir:
        command += " --sub-dir={}".format(sub_dir)

    return command


def main():
    if not 2 <= len(sys.argv) <= 3:
        return show_error("Usage: parse-install-yaml install.yaml [--now]")

    now = False
    if len(sys.argv) == 3:
        if sys.argv[2] == "--now":
            now = True
        else:
            return show_error("Unknown option: {0}".format(sys.argv[2]))

    with open(sys.argv[1]) as f:
        try:
            install_items = yaml.load(f, yaml.CSafeLoader)
        except AttributeError:
            install_items = yaml.load(f, yaml.SafeLoader)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            return show_error("Invalid yaml syntax: {0}".format(e))

    if not isinstance(install_items, list):
        return show_error("Root of install.yaml file should be a YAML sequence")

    commands = []

    # Combine now calls
    now_cache = {
        "system-now": [],
        "pip-now": [],
        "pip3-now": [],
        "ppa-now": [],
        "snap-now": [],
    }

    for install_item in install_items:
        command = None

        try:
            install_type = install_item["type"]

            if install_type == "empty":
                return 0

            if install_type == "ros":
                if "source" in install_item:
                    source = install_item["source"]
                else:
                    if ros_release in install_item:
                        source = install_item[ros_release]["source"]
                    elif "default" in install_item:
                        source = install_item["default"]["source"]
                    else:
                        return show_error(
                            "ROS distro {} or 'default' not specified in install.yaml".format(ros_release)
                        )
                    # Both release and default are allowed to be None
                    if source is None:
                        continue

                source_type = source["type"]
                if source_type == "git":
                    command = "tue-install-ros {}".format(catkin_git(source))
                elif source_type == "system":
                    command = "tue-install-ros system {0}".format(source["name"])
                else:
                    return show_error("Unknown ROS install type: '{0}'".format(source_type))

            # Non ros targets that are packaged to be built with catkin
            elif install_type == "catkin":
                source = install_item["source"]

                if not source["type"] == "git":
                    return show_error("Unknown catkin install type: '{0}'".format(source["type"]))
                command = "tue-install-ros {}".format(catkin_git(source))

            elif install_type == "git":
                command = "tue-install-git {}".format(type_git(install_item))

            elif install_type in [
                "target",
                "system",
                "pip",
                "pip3",
                "ppa",
                "snap",
                "dpkg",
                "target-now",
                "system-now",
                "pip-now",
                "pip3-now",
                "ppa-now",
                "snap-now",
            ]:
                if now and "now" not in install_type:
                    install_type += "-now"

                if "name" in install_item:
                    pkg_name = install_item["name"]
                else:
                    if ubuntu_release in install_item:
                        pkg_name = install_item[ubuntu_release]["name"]
                    elif "default" in install_item:
                        pkg_name = install_item["default"]["name"]
                    else:
                        return show_error(
                            "Ubuntu distro {} or 'default' not specified in install.yaml".format(ubuntu_release)
                        )
                    # Both release and default are allowed to be None
                    if pkg_name is None:
                        continue

                # Cache install types which accept multiple pkgs at once
                if install_type in now_cache:
                    now_cache[install_type].append(pkg_name)
                    continue

                command = "tue-install-{0} {1}".format(install_type, pkg_name)

            else:
                return show_error("Unknown install type: '{0}'".format(install_type))

        except KeyError as e:
            return show_error("Invalid install.yaml file: Key {0} could not be found.".format(e))

        if not command:
            return show_error("Invalid install.yaml file")

        command = command.replace(" ", "^")
        commands.append(command)

    for install_type, pkg_list in now_cache.items():
        if pkg_list:
            command = "tue-install-{0} {1}".format(install_type, " ".join(pkg_list))
            command = command.replace(" ", "^")
            commands.append(command)

    commands = " ".join(commands)

    print(commands)

    return 0


if __name__ == "__main__":
    sys.exit(main())
