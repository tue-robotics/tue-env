#! /usr/bin/env python

from __future__ import print_function

from os import environ
import sys
import yaml


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
                    release = environ["ROS_DISTRO"]
                    if release in install_item:
                        source = install_item[release]["source"]
                    elif "default" in install_item:
                        source = install_item["default"]["source"]
                    else:
                        return show_error("ROS distro {} or 'default' not specified in install.yaml".format(release))

                source_type = source["type"]
                if source_type == "svn" or source_type == "git":
                    if "sub-dir" in source:
                        sub_dir = source["sub-dir"]
                    else:
                        sub_dir = "."

                    command = "tue-install-{0} {1} {2} {3}".format(install_type, source_type, source["url"], sub_dir)
                    if "version" in source:
                        command += " {0}".format(source["version"])
                elif source_type == "system":
                    command = "tue-install-ros system {0}".format(source["name"])
                else:
                    return show_error("Unknown ROS install type: '{0}'".format(source_type))
            elif install_type == "target":
                command = "tue-install-target {0}".format(install_item["name"])
            elif install_type == "system":
                command = "tue-install-system {0}".format(install_item["name"])
            elif install_type == "pip":
                command = "tue-install-pip {0}".format(install_item["name"])
            elif install_type == "pip3":
                command = "tue-install-pip3 {0}".format(install_item["name"])
            elif install_type == "ppa":
                command = "tue-install-ppa {0}".format(install_item["name"])
            elif install_type == "snap":
                command = "tue-install-snap {0}".format(install_item["name"])
            elif install_type == "dpkg":
                command = "tue-install-dpkg {0}".format(install_item["name"])
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


if __name__ == "__main__":
    sys.exit(main())
