from typing import List, Optional

from contextlib import contextmanager
import datetime
import os
from pathlib import Path
import shlex
import subprocess
from termcolor import colored

from .install_yaml_parser import installyaml_parser

CI = None


def is_CI() -> bool:
    global CI
    if CI is None:
        CI = os.environ.get("CI", False)

    return CI


def date_stamp() -> str:
    return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")


class InstallerImpl:
    def __init__(self, debug: bool = False):
        self._debug = debug
        self._current_target = "main-loop"
        self._current_target_dir = ""
        stamp = date_stamp()
        self._log_file = os.path.join(os.sep, "tmp", f"tue-get-details-{stamp}")
        Path(self._log_file).touch(exist_ok=False)

        try:
            self._tue_env_dir = os.environ["TUE_ENV_DIR"]
        except KeyError:
            print("'TUE_ENV_DIR' is not defined")
            raise

        self._tue_apt_get_updated_file = os.path.join(os.sep, "tmp", "tue_get_apt_get_updated")

        self._dependencies_dir = os.path.join(self._tue_env_dir, ".env", "dependencies")
        self._dependencies_on_dir = os.path.join(self._tue_env_dir, ".env", "dependencies-on")
        self._installed_dir = os.path.join(self._tue_env_dir, ".env", "installed")

        os.makedirs(self._dependencies_dir, exist_ok=True)
        os.makedirs(self._dependencies_on_dir, exist_ok=True)
        os.makedirs(self._installed_dir, exist_ok=True)

        try:
            self._targets_dir = os.environ["TUE_ENV_TARGETS_DIR"]
        except KeyError:
            print("'TUE_ENV_TARGETS_DIR' is not defined")
            raise

        if not os.path.isdir(self._targets_dir):
            raise RuntimeError(f"TUE_INSTALL_TARGETS_DIR '{self._targets_dir}' does not exist as directory")

        general_state_dir = os.path.join(os.sep, "tmp", "tue-installer")
        if not os.path.isdir(general_state_dir):
            self.tue_install_debug(f"mkdir {general_state_dir}")
            os.makedirs(general_state_dir, mode=0o700, exist_ok=True)
            # self.tue_install_debug(f"chmod a+rwx {general_state_dir}")
            # os.chmod(general_state_dir, 0o700)

        self._state_dir = os.path.join(general_state_dir, stamp)
        self.tue_install_debug(f"mkdir {self._state_dir}")
        os.makedirs(self._state_dir, mode=0o700)

        self._warn_logs = []
        self._info_logs = []

    def __del__(self):
        os.rmdir(self._state_dir)

    def _log_to_file(self, msg: str) -> None:
        with open(self._log_file, "a") as f:
            f.write(msg + "\n")

    def _write_sorted_deps(self, child: str) -> None:
        self._write_sorted_dep_file(os.path.join(self._dependencies_dir, self._current_target), child)
        self._write_sorted_dep_file(os.path.join(self._dependencies_on_dir, child), self._current_target)

    def _write_sorted_dep_file(self, file: str, target: str) -> None:
        if not os.path.isfile(file):
            Path(file).touch(exist_ok=False)
            targets = []
        else:
            with open(file, "r+") as f:
                targets = f.readlines()

        print(f"{targets=}")
        targets.append(target + "\n")
        targets.sort()
        with open(file, "w") as f:
            f.writelines(targets)

    @contextmanager
    def _set_target(self, target: str):
        parent_target = self._current_target
        parent_target_dir = self._current_target_dir
        self._current_target = target
        self._current_target_dir = os.path.join(self._targets_dir, target)
        yield
        self._current_target = parent_target
        self._current_target_dir = parent_target_dir

    def tue_install_error(self, msg: str) -> None:
        log_msg = f"Error while installing target '{self._current_target}':\n\n\t{msg}\n\nLogfile: {self._log_file}"
        print(colored(log_msg, color="red"))
        self._log_to_file(log_msg)

    def tue_install_warning(self, msg: str) -> None:
        log_msg = f"[{self._current_target}] WARNING: {msg}"
        colored_log = colored(log_msg, color="yellow", attrs=["bold"])
        self._warn_logs.append(colored_log)
        self._log_to_file(log_msg)

    def tue_install_info(self, msg: str) -> None:
        log_msg = f"[{self._current_target}] INFO: {msg}"
        colored_log = colored(log_msg, color="cyan")
        self._info_logs.append(colored_log)
        print(colored_log)
        self._log_to_file(log_msg)

    def tue_install_debug(self, msg: str) -> None:
        log_msg = f"[{self._current_target}] DEBUG: {msg}"
        colored_log = colored(log_msg, color="blue")
        if self._debug:
            print(colored_log)
        self._log_to_file(log_msg)

    def tue_install_echo(self, msg: str) -> None:
        log_msg = f"[{self._current_target}]: {msg}"
        colored_log = colored(log_msg, attrs=["bold"])
        print(colored_log)
        self._log_to_file(log_msg)

    def tue_install_tee(self, msg: str) -> None:
        print(msg)
        self._log_to_file(msg)

    def tue_install_pipe(self):
        # ToDo: This should be implemented for install.bash scripts, not needed anymore for system calls in this script
        pass

    def tue_install_target_now(self, target: str) -> int:
        self.tue_install_debug(f"tue-install-target-now {target}")

        self.tue_install_debug(f"calling: tue-install-target {target} true")
        return self.tue_install_target(target, True)

    def tue_install_target(self, target: str, now: bool = False) -> int:
        self.tue_install_debug(f"tue-install-target {target} {now}")

        self.tue_install_debug("Installing target: {target}")

        # Check if valid target received as input
        if not os.path.isdir(os.path.join(self._targets_dir, target)):
            self.tue_install_debug(f"Target '{target}' does not exist.")
            return False

        # If the target has a parent target, add target as a dependency to the parent target
        if self._current_target and self._current_target != "main-loop" and self._current_target != target:
            self._write_sorted_deps(target)

        with self._set_target(target):

            state_file = os.path.join(self._state_dir, "target")
            state_file_now = f"{state_file}-now"

            # Determine if this target needs to be executed
            execution_needed = True

            if is_CI() and not os.path.isfile(os.path.join(self._current_target_dir, ".ci_ignore")):
                self.tue_install_debug(
                    f"Running installer in CI mode and file {self._current_target_dir}/.ci_ignore exists. No execution is needed."
                )
                execution_needed = False
            elif os.path.isfile(state_file_now):
                self.tue_install_debug(
                    f"File {state_file_now} does exist, so installation has already been executed with 'now' option. No execution is needed."
                )
                execution_needed = False
            elif os.path.isfile(state_file):
                if now:
                    self.tue_install_debug(
                        f"File {state_file_now} doesn't exist, but file {state_file} does. So installation has been executed yet, but not with the 'now' option. Going to execute it with 'now' option."
                    )
                else:
                    self.tue_install_debug(
                        f"File {state_file_now} does exist. 'now' is not enabled, so no execution needed."
                    )
                    execution_needed = False
            else:
                if now:
                    self.tue_install_debug(
                        f"Files {state_file_now} and {state_file} don't exist. Going to execute with 'now' option."
                    )
                else:
                    self.tue_install_debug(
                        f"Files {state_file_now} and {state_file} don't exist. Going to execute without 'now' option."
                    )

            if execution_needed:
                self.tue_install_debug("Starting installation")
                install_file = os.path.join(self._current_target_dir, "install")

                # Empty the target's dependency file
                dep_file = os.path.join(self._dependencies_dir, target)
                self.tue_install_debug(f"Emptying {dep_file}")
                if os.path.isfile(dep_file):
                    with open(dep_file, "a") as f:
                        f.truncate(0)

                target_processed = False

                install_yaml_file = install_file + ".yaml"
                if os.path.isfile(install_yaml_file):
                    if is_CI() and os.path.isfile(os.path.join(self._current_target_dir, ".ci_ignore_yaml")):
                        self.tue_install_debug(
                            "Running in CI mode and found .ci_ignore_yaml file, so skipping install.yaml"
                        )
                        target_processed = True
                    else:
                        self.tue_install_debug(f"Parsing {install_yaml_file}")

                        cmds = installyaml_parser(self, install_yaml_file, now)["commands"]
                        if not cmds:
                            self.tue_install_error(f"Invalid install.yaml: {cmds}")

                        for cmd in cmds:
                            self.tue_install_debug(str(cmd))
                            cmd() or self.tue_install_error(
                                f"Error while running: {cmd}"
                            )  # ToDo: Fix exception/return value handling

                        target_processed = True

                install_bash_file = install_file + ".bash"
                if os.path.isfile(install_bash_file):
                    if is_CI() and os.path.isfile(os.path.join(self._current_target_dir, ".ci_ignore_bash")):
                        self.tue_install_debug(
                            "Running in CI mode and found .ci_ignore_bash file, so skipping install.bash"
                        )
                    else:
                        self.tue_install_debug(f"Sourcing {install_bash_file}")
                        # ToDo: source install.bash

                    target_processed = True

                if not target_processed:
                    self.tue_install_warning("Target does not contain a valid install.yaml/bash file")

                if now:
                    state_file_to_touch = state_file_now
                else:
                    state_file_to_touch = state_file

                Path(state_file_to_touch).touch()

        self.tue_install_debug(f"Finished installing {target}")

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


if __name__ == "__main__":
    cmd = shlex.split("git -C /home/amigo/.tue status")
    p1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return_value = p1.wait()
    print(f"{return_value=}")
    print(f"{p1.stdout.readlines()=}")
    print(f"{p1.stderr.readlines()=}")
    print(date_stamp())
