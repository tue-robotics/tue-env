#! /usr/bin/env python3

from typing import Mapping
import os
import sys
import xml.etree.ElementTree as ET

from catkin_pkg.condition import evaluate_condition


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: parse_package_xml PACKAGE.XML")
        return 1

    print("\n".join(packagexml_parser(sys.argv[1])["deps"]))
    return 0


def packagexml_parser(path: str) -> Mapping:
    tree = ET.parse(path)
    doc = tree.getroot()

    dep_set = set()

    dep_types = []
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

    if os.getenv("TUE_INSTALL_TEST_DEPEND", "false") == "true":
        dep_types.append("test_depend")

    if os.getenv("TUE_INSTALL_DOC_DEPEND", "false") == "true":
        dep_types.append("doc_depend")

    for types in fields + dep_types:
        parsed[types] = set()

    for dep_type in dep_types:
        deps = doc.findall(dep_type)
        parsed[dep_type] |= {
            dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)
        }
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
