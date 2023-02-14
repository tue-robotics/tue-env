from typing import List, Mapping, Optional, Tuple

from catkin_pkg.package import Dependency, Package, parse_package
import os


def catkin_package_parser(
    path: str,
    skip_normal_deps: bool = False,
    test_deps: bool = False,
    doc_deps: bool = False,
    warnings: Optional[List[str]] = None,
    context: Optional[Mapping] = None,
) -> Tuple[Package, List[Dependency]]:
    if context is None:
        context = os.environ
    dep_types = []
    if not skip_normal_deps:
        dep_types.extend(
            [
                "build_depends",
                "buildtool_depends",
                "build_export_depends",
                "buildtool_export_depends",
                "exec_depends",
            ]
        )

    if test_deps:
        dep_types.append("test_depends")

    if doc_deps:
        dep_types.append("doc_depends")

    pkg = parse_package(path, warnings=warnings)
    pkg.evaluate_conditions(context)

    deps = []
    for dep_type in dep_types:
        dep_list = getattr(pkg, dep_type)
        for dep in dep_list:  # type: Dependency
            if dep.evaluated_condition is None:
                raise RuntimeError("Dependency condition is None")
            if dep.evaluated_condition:
                deps.append(dep)

    return pkg, deps
