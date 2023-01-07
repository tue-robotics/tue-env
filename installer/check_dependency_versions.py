#! /usr/bin/env python3

from typing import Mapping
import os
import sys
import xml.etree.ElementTree as ET

from catkin_pkg.condition import evaluate_condition

import rospkg

rospack = rospkg.RosPack()
from collections import namedtuple

Dependency = namedtuple("Dependency", ["name", "req_version", "actual_version", "comparator"])


def main() -> int:
    """
    Print all dependencies of the specified package
    """

    if len(sys.argv) != 2:
        print("Usage: parse_package_xml_2 PACKAGE.XML")
        return 1

    mismatch_found = False
    deps = recursive_get_deps(sys.argv[1])
    for key, value in deps.items():
        if value.req_version != "None" and value.req_version != value.actual_version:
            print(f"Package {value.name}, current version {value.actual_version}, required version {value.req_version}")
            mismatch_found = True
        # print("\n".join(dep))

    if mismatch_found:
        print("Mismatch found in dependencies")
    else:
        print("Dependencies are consistent")

    return 0


def recursive_get_deps(path: str) -> dict:
    parsed_packages = list()
    to_parse_packages = list()
    deps = dict()

    parsed_mapping = packagexml_parser(path)
    direct_deps = parsed_mapping["deps"]
    to_parse_packages.extend([dep[0] for dep in direct_deps])
    for dep in direct_deps:
        deps[dep[0]] = Dependency(dep[0], dep[1], "0.0.0", 0)

    while len(to_parse_packages) > 0:
        package = to_parse_packages.pop()
        parsed_packages.append(package)

        # get dependencies of the dependency
        try:
            dep_path = rospack.get_path(package) + "/package.xml"
        except rospkg.common.ResourceNotFound:
            # print(f"could not find {package}, assuming not a ros package")
            del deps[package]
            continue

        parsed_mapping = packagexml_parser(dep_path)
        # register current version
        version_set = parsed_mapping["version"]
        if len(version_set) != 1:
            print(f"Package {package} should have 1 version, instead its version is {version_set}")
            raise Exception  # TODO better exception
        # update actual version
        deps[package] = Dependency(
            deps[package].name, deps[package].req_version, list(version_set)[0], deps[package].comparator
        )

        dep_set = parsed_mapping["deps"]
        for dep in dep_set:
            if dep[0] in parsed_packages:
                if dep[0] in deps and deps[dep[0]].req_version != dep[1]:
                    print(
                        f"inconsistent dependency. According to package {package}, package {dep[0]} should have version {dep[1]}, but another package requires {deps[dep[0]].req_version}"
                    )
                continue
            if dep[0] not in to_parse_packages:
                to_parse_packages.append(dep[0])
                deps[dep[0]] = Dependency(dep[0], dep[1], "Problems!", 0)
    return deps


def packagexml_parser(path: str) -> Mapping:
    """
    parse the xml file and return a list of dependencies as a mapping
    @param path: absolute path to the xml file to be parsed
    @return Mapping with the following fields:
        parsed: ...
        deps: list of strings naming the dependencies of the package
    """
    tree = ET.parse(path)
    doc = tree.getroot()

    dep_set = set()

    dep_types = []
    dep_versions = []
    fields = ["name", "version", "description", "maintainer", "export"]
    parsed = {}

    if os.getenv("TUE_INSTALL_SKIP_ROS_DEPS", "false") == "false":
        dep_types.extend(
            [
                "build_depend",
                "buildtool_depend",
                "build_export_depend",
                "buildtool_export_depend",
                "exec_depend",
                "depend",
                "run_depend",
            ]
        )
        dep_versions.extend(["version_eq"])

    if os.getenv("TUE_INSTALL_TEST_DEPEND", "false") == "true":
        dep_types.append("test_depend")

    if os.getenv("TUE_INSTALL_DOC_DEPEND", "false") == "true":
        dep_types.append("doc_depend")

    for types in fields + dep_types:
        parsed[types] = set()

    for dep_type in dep_types:
        deps = doc.findall(dep_type)
        for dep in deps:
            if evaluate_condition(dep.attrib.get("condition", None), os.environ):
                version = None
                for dep_version in dep_versions:
                    version = dep.get(dep_version)
                version = str(version)
                parsed[dep_type] |= {(dep.text, version)}
        dep_set |= parsed[dep_type]

    for field in fields:
        values = doc.findall(field)
        if field == "export":
            for exports in values:
                parsed["build_type"] = {bt.text for bt in exports if bt.tag == "build_type"}
        else:
            parsed[field] |= {value.text for value in values}
            if field == "maintainer":
                emails = {}
                for value in values:
                    emails[value.text] = value.attrib.get("email")
                parsed["emails"] = emails

    return {"parser": parsed, "deps": dep_set, "version": parsed["version"]}


if __name__ == "__main__":
    sys.exit(main())
