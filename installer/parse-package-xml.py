#! /usr/bin/env python3

from __future__ import print_function

import os
import sys
import xml.etree.ElementTree as ET

from catkin_pkg.condition import evaluate_condition


def main():
    if len(sys.argv) != 2:
        print("Usage: parse-package-xml PACKAGE.XML")
        return 1

    tree = ET.parse(sys.argv[1])
    doc = tree.getroot()

    dep_set = set()

    deps = doc.findall('build_depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    deps = doc.findall('buildtool_depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    deps = doc.findall('run_depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    deps = doc.findall('depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    deps = doc.findall('build_export_depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    deps = doc.findall('buildtool_export_depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    deps = doc.findall('exec_depend')
    dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    if os.getenv('TUE_INSTALL_TEST_DEPEND', 'false') == 'true':
        deps = doc.findall('test_depend')
        dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    if os.getenv('TUE_INSTALL_DOC_DEPEND', 'false') == 'true':
        deps = doc.findall('doc_depend')
        dep_set |= {dep.text for dep in deps if evaluate_condition(dep.attrib.get("condition", None), os.environ)}

    print('\n'.join(dep_set))


if __name__ == "__main__":
    sys.exit(main())
