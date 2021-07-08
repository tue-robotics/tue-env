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
        "pip2-now": [],
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

            elif install_type == "ros":
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
                    sub_dir = source.get("sub-dir", ".")

                    command = "tue-install-ros {0} {1} {2}".format(source_type, source["url"], sub_dir)
                    if "version" in source:
                        command += " {0}".format(source["version"])
                elif source_type == "system":
                    command = "tue-install-ros system {0}".format(source["name"])
                else:
                    return show_error("Unknown ROS install type: '{0}'".format(source_type))

            elif install_type == "git":
                command = "tue-install-{0} {1} {2}".format(install_type, install_item["url"], install_item["path"])
                if "version" in install_item:
                    command += " {0}".format(install_item["version"])

            elif install_type in [
                "target",
                "system",
                "pip",
                "pip2",
                "pip3",
                "ppa",
                "snap",
                "dpkg",
                "target-now",
                "system-now",
                "pip-now",
                "pip2-now",
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
