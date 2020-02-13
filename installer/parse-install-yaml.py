#! /usr/bin/env python3

from __future__ import print_function

from os import environ
import sys
import yaml

from distro import linux_distribution

ros_release = environ["TUE_ROS_DISTRO"]
ubuntu_release = linux_distribution()[2]


def show_error(error):
    print("ERROR: {0}".format(error))
    return 1


def main():
    if len(sys.argv) != 2:
        print("Usage: parse-install-yaml install.yaml")
        return 1

    with open(sys.argv[1]) as f:
        try:
            install_items = yaml.safe_load(f)
        except (yaml.parser.ParserError, yaml.scanner.ScannerError) as e:
            return show_error("Invalid yaml syntax: {0}".format(e))

    if not isinstance(install_items, list):
        return show_error("Root of install.yaml file should be a YAML sequence")

    commands = []

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
                        return show_error("ROS distro {} or 'default' not specified in install.yaml".
                                          format(ros_release))
                    # Both release and default are allowed to be None
                    if source is None:
                        continue

                source_type = source["type"]
                if source_type == "svn" or source_type == "git" or source_type == "hg":
                    sub_dir = source.get("sub-dir", ".")

                    command = "tue-install-ros {0} {1} {2}".format(source_type, source["url"], sub_dir)
                    if "version" in source:
                        command += " {0}".format(source["version"])
                elif source_type == "system":
                    command = "tue-install-ros system {0}".format(source["name"])
                else:
                    return show_error("Unknown ROS install type: '{0}'".format(source_type))

            elif install_type == "svn" or install_type == "git" or install_type == "hg":
                command = "tue-install-{0} {1} {2}".format(install_type, install_item["url"], install_item["path"])
                if "version" in install_item:
                    command += " {0}".format(install_item["version"])

            elif install_type == "target" or install_type == "system" or install_type == "pip" or \
                    install_type == "pip2" or install_type == "pip3" or install_type == "ppa" or \
                    install_type == "snap" or install_type == "dpkg":
                if "name" in install_item:
                    pkg_name = install_item["name"]
                else:
                    if ubuntu_release in install_item:
                        pkg_name = install_item[ubuntu_release]["name"]
                    elif "default" in install_item:
                        pkg_name = install_item["default"]["name"]
                    else:
                        return show_error("Ubuntu distro {} or 'default' not specified in install.yaml".
                                          format(ubuntu_release))
                    # Both release and default are allowed to be None
                    if pkg_name is None:
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

    commands = " ".join(commands)

    print(commands)

    return 0


if __name__ == "__main__":
    sys.exit(main())
