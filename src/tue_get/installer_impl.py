import glob
import subprocess
from typing import List, Optional, Union

from contextlib import contextmanager
import datetime
import filecmp
import getpass
import os
from pathlib import Path
import shlex
import shutil
from termcolor import colored

from tue_get.install_yaml_parser import installyaml_parser
from tue_get.util import BackgroundPopen

CI = None


def is_CI() -> bool:
    global CI
    if CI is None:
        CI = os.environ.get("CI", False)

    return CI


def date_stamp() -> str:
    return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")


def _which_split_cmd(cmd: str) -> List[str]:
    cmds = shlex.split(cmd)
    cmds[0] = shutil.which(cmds[0])
    return cmds


class InstallerImpl:
    _apt_get_update_file = os.path.join(os.pathsep, "tmp", "tue_get_apt_get_updated")

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

        # ToDo: we might want to use sets here
        self._systems = []
        self._ppas = []
        self._pips = []
        self._snaps = []
        self._gems = []

        self._sudo_password = None

    def __del__(self):
        shutil.rmtree(self._state_dir)

    def _log_to_file(self, msg: str, end="\n") -> None:
        with open(self._log_file, "a") as f:
            f.write(msg + end)

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

    def _out_handler(self, sub: BackgroundPopen, line: Union[bytes, str]) -> None:
        def _write_stdin(msg) -> None:
            sub.stdin.write(f"{msg}\n")
            sub.stdin.flush()

        line = line.strip()
        # ToDo: use regex to get attributes of installer
        if line.startswith("[sudo] password for"):
            if self._sudo_password is None:
                self._sudo_password = getpass.getpass(line)
            _write_stdin(self._sudo_password)
        elif line.startswith("tue-install-error: "):
            self.tue_install_error(line[19:])
            _write_stdin(1)
        elif line.startswith("tue-install-warning: "):
            self.tue_install_warning(line[21:])
            _write_stdin(0)
        elif line.startswith("tue-install-info: "):
            self.tue_install_info(line[18:])
            _write_stdin(0)
        elif line.startswith("tue-install-debug: "):
            self.tue_install_debug(line[19:])
            _write_stdin(0)
        elif line.startswith("tue-install-echo: "):
            self.tue_install_echo(line[18:])
            _write_stdin(0)
        elif line.startswith("tue-install-tee: "):
            self.tue_install_tee(line[17:])
            _write_stdin(0)
        elif line.startswith("tue-install-pipe: "):
            output = line[18:].split("^^^")
            self.tue_install_pipe(*output)
            _write_stdin(0)
        elif line.startswith("tue-install-target-now: "):
            self.tue_install_target_now(line[24:])
            _write_stdin(0)
        elif line.startswith("tue-install-target: "):
            success = self.tue_install_target(line[20:])
            _write_stdin(not success)
        elif line.startswith("tue-install-git: "):
            success = self.tue_install_git(*line[17:].split())
            _write_stdin(not success)
        elif line.startswith("tue-install-apply-patch: "):
            success = self.tue_install_apply_patch(*line[25:].split())
            _write_stdin(not success)
        elif line.startswith("tue-install-cp: "):
            success = self.tue_install_cp(*line[15:].split())
            _write_stdin(not success)
        elif line.startswith("tue-install-add-text: "):
            success = self.tue_install_add_text(*line[21:].split())
            _write_stdin(not success)
        elif line.startswith("tue-install-get-releases: "):
            success = self.tue_install_get_releases(*line[26:].split())
            _write_stdin(not success)
        elif line.startswith("tue-install-system: "):
            pkgs = line[20:].split()
            self.tue_install_system(pkgs)
            _write_stdin(0)
        elif line.startswith("tue-install-system-now: "):
            pkgs = line[24:].split()
            success = self.tue_install_system_now(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-apt-get-update"):
            self.tue_install_apt_get_update()
            _write_stdin(0)
        elif line.startswith("tue-install-ppa: "):
            ppas = line[17:].split()
            self.tue_install_ppa(ppas)
            _write_stdin(0)
        elif line.startswith("tue-install-ppa-now: "):
            ppas = line[21:].split()
            success = self.tue_install_ppa_now(ppas)
            _write_stdin(not success)
        elif line.startswith("tue-install-pip: "):
            pkgs = line[17:].split()
            self.tue_install_pip(pkgs)
            _write_stdin(0)
        elif line.startswith("tue-install-pip-now: "):
            pkgs = line[20:].split()
            success = self.tue_install_pip_now(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-snap: "):
            pkgs = line[18:].split()
            self.tue_install_snap(pkgs)
            _write_stdin(0)
        elif line.startswith("tue-install-snap-now: "):
            pkgs = line[22:].split()
            success = self.tue_install_snap_now(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-gem: "):
            pkgs = line[17:].split(" ")
            self.tue_install_gem(pkgs)
            _write_stdin(0)
        elif line.startswith("tue-install-gem-now: "):
            pkgs = line[21:].split(" ")
            success = self.tue_install_gem_now(pkgs)
            _write_stdin(not success)
        else:
            self.tue_install_tee(line)

    def _err_handler_sudo_password(self, sub: BackgroundPopen, line: Union[bytes, str]) -> None:
        line = line.strip()
        if line.startswith("[sudo] password for"):
            if self._sudo_password is None:
                self._sudo_password = getpass.getpass(line)
            sub.stdin.write(f"{self._sudo_password}\n")
            sub.stdin.flush()
        else:
            self.tue_install_tee(line, color="red")

    def tue_install_error(self, msg: str) -> None:
        # Make sure the entire msg is indented, not just the first line
        lines = msg.splitlines()
        lines = [f"    {line}" for line in lines]
        msg = "\n".join(lines)
        log_msg = f"Error while installing target '{self._current_target}':\n\n{msg}\n\nLogfile: {self._log_file}"
        print(colored(log_msg, color="red"))
        self._log_to_file(log_msg)
        # ToDo: We should stop after this

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

    def tue_install_tee(self, msg: str, color=None, on_color=None, attrs=None) -> None:
        print(colored(msg, color=color, on_color=on_color, attrs=attrs))
        self._log_to_file(msg)

    def tue_install_pipe(self, stdout: str, stderr: str) -> None:
        """
        Only to be used for bash scripts
        Prints stdout, stderr is printed in red

        :param stdout:
        :param stderr:
        :return:
        """
        self.tue_install_tee(stdout)
        self.tue_install_tee(stderr, color="red")

    def tue_install_target_now(self, target: str) -> bool:
        self.tue_install_debug(f"tue-install-target-now {target=}")

        self.tue_install_debug(f"calling: tue-install-target {target} True")
        return self.tue_install_target(target, True)

    def tue_install_target(self, target: str, now: bool = False) -> bool:
        self.tue_install_debug(f"tue-install-target {target=} {now=}")

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
                                f"Error while running:\n    {cmd}"
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
                        resource_file = os.path.join(os.path.dirname(__file__), "resources", "installer_impl.bash")
                        cmd = f"bash -c 'source {resource_file} && source {install_bash_file}'"
                        cmds = _which_split_cmd(cmd)
                        self.tue_install_echo(" ".join(cmds))
                        sub = BackgroundPopen(
                            args=cmds,
                            out_handler=self._out_handler,
                            err_handler=self._err_handler_sudo_password,
                            stdin=subprocess.PIPE,
                            text=True,
                        )
                        sub.wait()
                        if sub.returncode != 0:
                            # stderr = sub.stderr.read()
                            self.tue_install_error(
                                f"Error while running({sub.returncode}):\n    {' '.join(cmds)}" f"\n\n{stderr}"
                            )
                            # ToDo: This depends on behaviour of tue-install-error
                            return False

                    target_processed = True

                if not target_processed:
                    self.tue_install_warning("Target does not contain a valid install.yaml/bash file")

                if now:
                    state_file_to_touch = state_file_now
                else:
                    state_file_to_touch = state_file

                Path(state_file_to_touch).touch()

        self.tue_install_debug(f"Finished installing {target}")

    def tue_install_git(self, url: str, target_dir: Optional[str] = None, version: Optional[str] = None) -> bool:
        pass

    def tue_install_apply_patch(self, patch_file: str, target_dir: str) -> bool:
        self.tue_install_debug(f"tue-install-apply-patch {patch_file=} {target_dir=}")

        if not patch_file:
            self.tue_install_error("Invalid tue-install-apply-patch call: needs patch file as argument")

        patch_file_path = os.path.join(self._current_target_dir, patch_file)
        if not os.path.isfile(patch_file_path):
            self.tue_install_error(f"Invalid tue-install-apply-patch call: patch file {patch_file_path} does not exist")

        if not target_dir:
            self.tue_install_error("Invalid tue-install-apply-patch call: needs target directory as argument")

        if not os.path.isdir(target_dir):
            self.tue_install_error(
                f"Invalid tue-install-apply-patch call: target directory {target_dir} does not exist"
            )

        cmd = f"patch -s -N -r - -p0 -d {target_dir} < {patch_file_path}"
        cmds = _which_split_cmd(cmd)
        self.tue_install_echo(" ".join(cmds))
        sub = BackgroundPopen(args=cmds, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        sub.wait()
        if sub.returncode != 0:
            stderr = sub.stderr.read()
            self.tue_install_error(f"Error while running({sub.returncode}):\n{' '.join(cmds)}\n\n{stderr}")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        return True

    def tue_install_cp(self, source: str, target: str) -> bool:
        self.tue_install_debug(f"tue-install-cp {source=} {target=}")

        if not source:
            self.tue_install_error("Invalid tue-install-cp call: needs source directory as argument")

        if not target:
            self.tue_install_error("Invalid tue-install-cp call: needs target dir as argument")

        if os.path.isdir(target):
            self.tue_install_debug(f"tue-install-cp: target {target} is a directory")
        elif os.path.isfile(target):
            self.tue_install_debug(f"tue-install-cp: target {target} is a file")
            target = os.path.dirname(target)
        else:
            self.tue_install_error(f"tue-install-cp: target {target} does not exist")

        # Check if user is allowed to write on target destination
        root_required = True

        if os.access(target, os.W_OK):
            root_required = False

        source_path = os.path.join(self._current_target_dir, source)
        source_files = glob.glob(source_path)

        for file in source_files:
            if not os.path.isfile(file):
                self.tue_install_error(f"tue-install-cp: source {file} is not a file")

            cp_target = os.path.join(target, os.path.basename(file))

            if os.path.isfile(cp_target) and filecmp.cmp(file, cp_target):
                self.tue_install_debug(f"tue-install-cp: {file} and {cp_target} are identical, skipping")
                continue

            self.tue_install_debug(f"tue-install-cp: copying {file} to {cp_target}")

            if root_required:
                sudo_cmd = "sudo "
                self.tue_install_debug(f"Using elevated privileges (sudo) to copy ${file} to ${cp_target}")
            else:
                sudo_cmd = ""

            cmd = f"{sudo_cmd}python -c 'import os; os.makedirs(\"{target}\", exist_ok=True)'"
            cmds = _which_split_cmd(cmd)
            self.tue_install_echo(" ".join(cmds))
            sub = BackgroundPopen(
                args=cmds,
                err_handler=self._err_handler_sudo_password,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                text=True,
            )
            sub.wait()
            if sub.returncode != 0:
                # stderr = sub.stderr.read()
                self.tue_install_error(
                    f"Error while creating the directory({sub.returncode}):\n{' '.join(cmds)}" #  f"\n\n{stderr}"
                )
                # ToDo: This depends on behaviour of tue-install-error
                return False

            cmd = f'{sudo_cmd}python -c \'import shutil; shutil.copy2("{file}", "{cp_target}")\''
            cmds = _which_split_cmd(cmd)
            self.tue_install_echo(" ".join(cmds))
            sub = BackgroundPopen(
                args=cmds,
                err_handler=self._err_handler_sudo_password,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                text=True,
            )
            sub.wait()
            if sub.returncode != 0:
                stderr = sub.stderr.read()
                self.tue_install_error(f"Error while copying({sub.returncode}):\n{' '.join(cmds)}\n\n{stderr}")
                # ToDo: This depends on behaviour of tue-install-error
                return False

    def tue_install_add_text(self, source_file: str, target_file: str) -> bool:
        pass

    def tue_install_get_releases(self, url: str, filename: str, output_dir: str, tag: Optional[str] = None) -> bool:
        pass

    def tue_install_system(self, pkgs: List[str]):
        self._systems.extend(pkgs)

    def tue_install_system_now(self, pkgs: List[str]) -> bool:
        pass

    def tue_install_apt_get_update(self):
        self.tue_install_debug("tue-install-apt-get-update")
        self.tue_install_debug("Requiring an update of apt-get before next 'apt-get install'")
        if os.path.isfile(self._apt_get_updated_file):
            os.remove(self._apt_get_updated_file)

    def tue_install_ppa(self, ppas: List[str]):
        self._ppas.extend(ppas)

    def tue_install_ppa_now(self, ppa: List[str]) -> bool:
        pass

    def tue_install_pip(self, pkgs: List[str]):
        self._pips.extend(pkgs)

    def tue_install_pip_now(self, pkgs: List[str]) -> bool:
        pass

    def tue_install_snap(self, pkgs: List[str]):
        self._snaps.extend(pkgs)

    def tue_install_snap_now(self, pkgs: List[str]) -> bool:
        pass

    def tue_install_gem(self, pkgs: List[str]):
        self._gems.extend(pkgs)

    def tue_install_gem_now(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-gem-now {pkgs=}")

        if not pkgs:
            self.tue_install_error("Invalid tue-install-gem-now call: got an empty list of packages as argument.")

        self.tue_install_system_now(["ruby", "ruby-dev", "rubygems-integration"])

        cmd = "gem list"
        cmds = _which_split_cmd(cmd)
        self.tue_install_echo(" ".join(cmds))
        sub = BackgroundPopen(args=cmds, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        sub.wait()
        if sub.returncode != 0:
            stderr = sub.stderr.read()
            self.tue_install_error(
                f"Error while getting installed gem packages({sub.returncode}):\n    {' '.join(cmds)}\n\n{stderr}"
            )
            # ToDo: This depends on behaviour of tue-install-error
            return False

        gems_installed = sub.stdout.read().splitlines()
        gems_installed = [gem.split(" ")[0] for gem in gems_installed]
        gems_to_install = []
        for pkg in pkgs:
            if pkg not in gems_installed:
                gems_to_install.append(pkg)
                self.tue_install_debug(f"gem pkg: {pkg} is not yet installed")
            else:
                self.tue_install_debug(f"gem pkg: {pkg} is already installed")

        if gems_to_install:
            cmd = f"gem install {' '.join(gems_to_install)}"
            cmds = _which_split_cmd(cmd)
            sub = BackgroundPopen(args=cmds, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            sub.wait()
            if sub.returncode != 0:
                stderr = sub.stderr.read()
                self.tue_install_error(
                    f"An error occurred while installing gem packages({sub.returncode}):\n    {' '.join(cmds)}"
                    f"\n\n{stderr}"
                )
                # ToDo: This depends on behaviour of tue-install-error
                return False

        return True

    def tue_install_dpkg(self, pkg_file: str):
        pass

    def tue_install_dpkg_now(self, pkg_file: str):
        pass

    def tue_install_ros(self, source_type: str, **kwargs):
        pass


if __name__ == "__main__":
    bla = InstallerImpl(debug=True)
    bla.tue_install_target("test")
