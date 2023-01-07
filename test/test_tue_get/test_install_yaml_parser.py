import os
from typing import List, Optional

import tempfile
import yaml

from tue_get import install_yaml_parser


class MockInstaller:
    def __init__(self):
        pass

    def tue_install_target_now(self, target: str):
        pass

    def tue_install_target(self, target: str, now: bool = False):
        pass

    def tue_install_git(self, url: str, target_dir: Optional[str] = None, version: Optional[str] = None):
        pass

    def tue_install_apply_patch(self, patch_file: str, target_dir: str):
        pass

    def tue_install_cp(self, source_file: str, target_file: str):
        pass

    def tue_install_add_text(self, source_file: str, target_file: str):
        pass

    def tue_install_get_releases(self, url: str, filename: str, output_dir: str, tag: Optional[str] = None):
        pass

    def tue_install_system(self, pkgs: List[str]):
        pass

    def tue_install_system_now(self, pkgs: List[str]):
        pass

    def tue_install_apt_get_update(self):
        pass

    def tue_install_ppa(self, ppa: List[str]):
        pass

    def tue_install_ppa_now(self, ppa: List[str]):
        pass

    def tue_install_pip(self, pkgs: List[str]):
        pass

    def tue_install_pip_now(self, pkgs: List[str]):
        pass

    def tue_install_snap(self, pkgs: List[str]):
        pass

    def tue_install_snap_now(self, pkgs: List[str]):
        pass

    def tue_install_gem(self, pkgs: List[str]):
        pass

    def tue_install_gem_now(self, pkgs: List[str]):
        pass

    def tue_install_dpkg(self, pkg_file: str):
        pass

    def tue_install_dpkg_now(self, pkg_file: str):
        pass

    def tue_install_ros(self, source_type: str, **kwargs):
        pass


def parse_target(yaml_seq: List, now: bool = False):
    install_file = tempfile.NamedTemporaryFile(mode="w", delete=False)
    try:
        dumper = yaml.CSafeDumper
    except AttributeError:
        dumper = yaml.SafeDumper

    yaml.dump(yaml_seq, install_file, dumper)

    install_file.close()

    installer = MockInstaller()
    try:
        cmds = install_yaml_parser.installyaml_parser(installer, install_file.name, now)["commands"]
    except Exception:
        raise
    finally:
        os.unlink(install_file.name)

    return cmds


def test_empty_target():
    target = [{"type": "empty"}]
    cmds = parse_target(target, False)
    assert not cmds
    cmds = parse_target(target, True)
    assert not cmds


def test_system_target():
    target = [{"type": "system", "name": "pkg_name"}]
    cmds = parse_target(target, False)
    assert cmds
    assert len(cmds) == 1
    for cmd in cmds:
        cmd()
    # assert isinstance(cmds[0].func, MockInstaller.tue_install_system)
    cmds = parse_target(target, True)
    assert cmds
    assert len(cmds) == 1
    # assert isinstance(cmds[0].func, MockInstaller.tue_install_system_now)
