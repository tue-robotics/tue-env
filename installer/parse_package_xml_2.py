#! /usr/bin/env python3

from typing import Mapping
import os
import sys
import xml.etree.ElementTree as ET

from catkin_pkg.condition import evaluate_condition

import rospkg
rospack = rospkg.RosPack()

import copy


def main() -> int:
    """
    Print all dependencies of the specified package
    """

    if len(sys.argv) != 2:
        print("Usage: parse_package_xml_2 PACKAGE.XML")
        return 1

    deps = recursive_get_deps(sys.argv[1])
    for dep in deps:
        print(f"Dependency on {dep[0]} with version {dep[1]}")
        #print("\n".join(dep))
    return 0


def recursive_get_deps(path: str) -> set:
    parsed_packages = list()
    to_parse_packages = list()
    deps = set()

    direct_deps = packagexml_parser(path)["deps"]
    to_parse_packages.extend([dep[0] for dep in direct_deps])
    deps |= direct_deps

    while len(to_parse_packages) > 0:
        package = to_parse_packages.pop()
        parsed_packages.append(package)

        # get dependencies of the dependency
        try:
            dep_path = rospack.get_path(package) + "/package.xml"
        except rospkg.common.ResourceNotFound:
            print(f"could not find {package}")
            continue
        dep_set = packagexml_parser(dep_path)["deps"]
        for dep in dep_set:
            if dep[0] in parsed_packages:
                continue
            if dep[0] not in to_parse_packages:
                # TODO check if package dep version also matches the one in the set.
                to_parse_packages.append(dep[0])
                deps |= {dep}
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

    return {"parser": parsed, "deps": dep_set}


if __name__ == "__main__":
    sys.exit(main())
