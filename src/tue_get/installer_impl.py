from typing import List, Optional, Tuple

from contextlib import contextmanager
import datetime
import filecmp
import getpass
import glob
import os
from pathlib import Path
from pip import main as pip_main
from pip._internal.req.constructors import install_req_from_line as pip_install_req_from_line
import re
import shlex
import shutil
import subprocess as sp
from termcolor import colored
from time import sleep

from tue_get.install_yaml_parser import installyaml_parser
from tue_get.util.BackgroundPopen import BackgroundPopen
from tue_get.util.grep import grep_directory, grep_file

CI = None


def is_CI() -> bool:
    global CI
    if CI is None:
        CI = os.environ.get("CI", False)

    return CI


def date_stamp() -> str:
    return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")


def _which_split_cmd(cmd: str) -> Tuple[str, List[str]]:
    cmds = shlex.split(cmd)
    cmds[0] = shutil.which(cmds[0])
    return " ".join(cmds), cmds


def _wait_for_dpkg_lock():
    i = 0
    rotate_list = ["-", "\\", "|", "/"]
    cmd = "sudo fuser /var/lib/dpkg/lock"
    cmd, cmds = _which_split_cmd(cmd)
    while sp.run(cmds, stdout=sp.DEVNULL, stderr=sp.DEVNULL).returncode == 0:
        print(f"[{rotate_list[i % len(rotate_list)]}] Waiting for dpkg lock", end="\r")
        i += 1
        sleep(0.4)
    return


class InstallerImpl:
    _apt_get_updated_file = os.path.join(os.sep, "tmp", "tue_get_apt_get_updated")
    _sources_list = os.path.join(os.sep, "etc", "apt", "sources.list")
    _sources_list_dir = os.path.join(os.sep, "etc", "apt", "sources.list.d")

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

    @staticmethod
    def _write_sorted_dep_file(file: str, target: str) -> None:
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

    def _out_handler(self, sub: BackgroundPopen, line: str) -> None:
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
            # ToDo: or should we call sub.kill() here?
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
            output = map(lambda x: x.replace("^", "\n").strip(), output)
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
            success = self.tue_install_system(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-system-now: "):
            pkgs = line[24:].split()
            success = self.tue_install_system_now(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-apt-get-update"):
            self.tue_install_apt_get_update()
            _write_stdin(0)
        elif line.startswith("tue-install-ppa: "):
            ppas = line[17:].split()
            ppas = [ppa.replace("^", " ").strip() for ppa in ppas]
            success = self.tue_install_ppa(ppas)
            _write_stdin(not success)
        elif line.startswith("tue-install-ppa-now: "):
            ppas = line[21:].split()
            ppas = [ppa.replace("^", " ").strip() for ppa in ppas]
            success = self.tue_install_ppa_now(ppas)
            _write_stdin(not success)
        elif line.startswith("tue-install-pip: "):
            pkgs = line[17:].split()
            success = self.tue_install_pip(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-pip-now: "):
            pkgs = line[20:].split()
            success = self.tue_install_pip_now(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-snap: "):
            pkgs = line[18:].split()
            success = self.tue_install_snap(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-snap-now: "):
            pkgs = line[22:].split()
            success = self.tue_install_snap_now(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-gem: "):
            pkgs = line[17:].split(" ")
            success = self.tue_install_gem(pkgs)
            _write_stdin(not success)
        elif line.startswith("tue-install-gem-now: "):
            pkgs = line[21:].split(" ")
            success = self.tue_install_gem_now(pkgs)
            _write_stdin(not success)
        else:
            self.tue_install_tee(line)

    def _err_handler(self, sub: BackgroundPopen, line: str) -> None:
        line = line.strip()
        if line.startswith("[sudo] password for"):
            if self._sudo_password is None:
                self._sudo_password = getpass.getpass(line)
            sub.stdin.write(f"{self._sudo_password}\n")
            sub.stdin.flush()
        else:
            self.tue_install_tee(line, color="red")

    def _default_background_popen(self, cmd: str) -> BackgroundPopen:
        cmd, cmds = _which_split_cmd(cmd)
        self.tue_install_echo(repr(cmd))
        sub = BackgroundPopen(
            args=cmds, out_handler=self._out_handler, err_handler=self._err_handler, stdin=sp.PIPE, text=True
        )
        sub.wait()
        return sub

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
        if stdout:
            self.tue_install_tee(stdout)
        if stderr:
            self.tue_install_tee(stderr, color="red")

    def tue_install_target_now(self, target: str) -> bool:
        self.tue_install_debug(f"tue-install-target-now {target=}")

        self.tue_install_debug(f"calling: tue-install-target {target} True")
        return self.tue_install_target(target, True)

    def tue_install_target(self, target: str, now: bool = False) -> bool:
        self.tue_install_debug(f"tue-install-target {target=} {now=}")

        self.tue_install_debug(f"Installing target: {target}")

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
                    f"Running installer in CI mode and file {self._current_target_dir}/.ci_ignore exists. "
                    "No execution is needed."
                )
                execution_needed = False
            elif os.path.isfile(state_file_now):
                self.tue_install_debug(
                    f"File {state_file_now} does exist, so installation has already been executed with 'now' option. "
                    "No execution is needed."
                )
                execution_needed = False
            elif os.path.isfile(state_file):
                if now:
                    self.tue_install_debug(
                        f"File {state_file_now} doesn't exist, but file {state_file} does. "
                        "So installation has been executed yet, but not with the 'now' option. "
                        "Going to execute it with 'now' option."
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
                self.tue_install_debug(f"Starting installation of target: {target}")
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
                            # ToDo: This depends on behaviour of tue-install-error
                            return False

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
                        cmd = f'bash -c "source {resource_file} && source {install_bash_file}"'
                        sub = self._default_background_popen(cmd)
                        if sub.returncode != 0:
                            self.tue_install_error(f"Error while running({sub.returncode}):\n    {repr(cmd)}")
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
            # ToDo: This depends on behaviour of tue-install-error
            return False

        patch_file_path = os.path.join(self._current_target_dir, patch_file)
        if not os.path.isfile(patch_file_path):
            self.tue_install_error(f"Invalid tue-install-apply-patch call: patch file {patch_file_path} does not exist")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if not target_dir:
            self.tue_install_error("Invalid tue-install-apply-patch call: needs target directory as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if not os.path.isdir(target_dir):
            self.tue_install_error(
                f"Invalid tue-install-apply-patch call: target directory {target_dir} does not exist"
            )
            # ToDo: This depends on behaviour of tue-install-error
            return False

        cmd = f"patch -s -N -r - -p0 -d {target_dir} < {patch_file_path}"
        sub = self._default_background_popen(cmd)
        if sub.returncode != 0:
            self.tue_install_error(f"Error while running({sub.returncode}):\n    {repr(cmd)}")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        return True

    def tue_install_cp(self, source: str, target: str) -> bool:
        self.tue_install_debug(f"tue-install-cp {source=} {target=}")

        if not source:
            self.tue_install_error("Invalid tue-install-cp call: needs source directory as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if not target:
            self.tue_install_error("Invalid tue-install-cp call: needs target dir as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if os.path.isdir(target):
            self.tue_install_debug(f"tue-install-cp: target {target} is a directory")
        elif os.path.isfile(target):
            self.tue_install_debug(f"tue-install-cp: target {target} is a file")
            target = os.path.dirname(target)
        else:
            self.tue_install_error(f"tue-install-cp: target {target} does not exist")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        # Check if user is allowed to write on target destination
        root_required = True

        if os.access(target, os.W_OK):
            root_required = False

        source_path = os.path.join(self._current_target_dir, source)
        source_files = glob.glob(source_path)

        for file in source_files:
            if not os.path.isfile(file):
                self.tue_install_error(f"tue-install-cp: source {file} is not a file")
                # ToDo: This depends on behaviour of tue-install-error
                return False

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
            sub = self._default_background_popen(cmd)
            if sub.returncode != 0:
                self.tue_install_error(f"Error while creating the directory({sub.returncode}):\n    {repr(cmd)}")
                # ToDo: This depends on behaviour of tue-install-error
                return False

            cmd = f'{sudo_cmd}python -c \'import shutil; shutil.copy2("{file}", "{cp_target}")\''
            sub = self._default_background_popen(cmd)
            if sub.returncode != 0:
                self.tue_install_error(f"Error while copying({sub.returncode}):\n    {repr(cmd)}")
                # ToDo: This depends on behaviour of tue-install-error
                return False

    def tue_install_add_text(self, source_file: str, target_file: str) -> bool:
        self.tue_install_debug(f"tue-install-add-text {source_file=} {target_file=}")

        if not source_file:
            self.tue_install_error("Invalid tue-install-add-text call: needs source file as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if not target_file:
            self.tue_install_error("Invalid tue-install-add-text call: needs target file as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if source_file.startswith("/") or source_file.startswith("~"):
            self.tue_install_error(
                "Invalid tue-install-add-text call: source file must be relative to target directory"
            )
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if not target_file.startswith("/") and not target_file.startswith("~"):
            self.tue_install_error(
                "Invalid tue-install-add-text call: target file must be absolute or " "relative to the home directory"
            )
            # ToDo: This depends on behaviour of tue-install-error
            return False

        root_required = True
        if os.access(target_file, os.W_OK):
            root_required = False
            self.tue_install_debug("tue-install-add-text: NO root required")
        else:
            self.tue_install_debug("tue-install-add-text: root required")

        if root_required:
            sudo_cmd = "sudo "
            self.tue_install_debug("Using elevated privileges (sudo) to add text")
        else:
            sudo_cmd = ""

        source_file_path = os.path.join(self._current_target_dir, source_file)
        if not os.path.isfile(source_file_path):
            self.tue_install_error(f"tue-install-add-text: source file {source_file_path} does not exist")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        target_file_path = os.path.expanduser(target_file)
        if not os.path.isfile(target_file_path):
            self.tue_install_error(f"tue-install-add-text: target file {target_file_path} does not exist")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        with open(source_file_path, "r") as f:
            source_text = f.read().splitlines()

        begin_tag = source_text[0]
        end_tag = source_text[-1]
        source_body = source_text[1:-1]

        self.tue_install_debug(f"tue-install-add-text: {begin_tag=}, {end_tag=}\n{source_body=}")

        with open(target_file_path, "r") as f:
            target_text = f.read().splitlines()

        if begin_tag not in target_text:
            self.tue_install_debug(
                f"tue-install-add-text: {begin_tag=} not found in {target_file_path=}, "
                "appending to {target_file_path}"
            )
            source_text = "\n".join(source_text)
            cmd = f"bash -c \"echo - e '{source_text}' | {sudo_cmd}tee -a {target_file_path}\""
        else:
            self.tue_install_debug(
                f"tue-install-add-text: {begin_tag=} found in {target_file_path=}, "
                "so comparing the files for changed lines"
            )

            begin_index = target_text.index(begin_tag)
            end_index = target_text.index(end_tag)
            target_body = target_text[begin_index + 1 : end_index]
            self.tue_install_debug(f"tue-install-add-text:\n{target_body=}")

            if source_body == target_body:
                self.tue_install_debug("tue-install-add-text: Lines have not changed, so not copying")
                return True

            self.tue_install_debug("tue-install-add-text: Lines have changed, so copying")

            target_text = target_text[: begin_index + 1] + source_body + target_text[end_index:]
            print(target_text)
            target_text = "\n".join(target_text)

            cmd = f"bash -c \"echo -e '{target_text}' | {sudo_cmd}tee {target_file_path}\""

        cmd, cmds = _which_split_cmd(cmd)
        self.tue_install_echo(repr(cmd))
        sub = BackgroundPopen(
            args=cmds,
            err_handler=self._err_handler,
            stdout=sp.DEVNULL,
            stdin=sp.PIPE,
            text=True,
        )
        sub.wait()
        if sub.returncode != 0:
            self.tue_install_error(f"Error while adding text({sub.returncode}):\n    {repr(cmd)}")
            # ToDo: This depends on behaviour of tue-install-error
            return False

    def tue_install_get_releases(self, url: str, filename: str, output_dir: str, tag: Optional[str] = None) -> bool:
        return True

    def tue_install_system(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-system {pkgs=}")
        if not pkgs:
            self.tue_install_error("Invalid tue-install-system call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self._systems.extend(pkgs)
        return True

    def tue_install_system_now(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-system-now {pkgs=}")
        if not pkgs:
            self.tue_install_error("Invalid tue-install-system-now call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        installed_pkgs = []

        def _out_handler_installed_pkgs(sub: BackgroundPopen, line: str) -> None:
            installed_pkgs.append(line.strip())

        # Check if pkg is not already installed dpkg -S does not cover previously removed packages
        # Based on https://stackoverflow.com/questions/1298066
        cmd = "dpkg-query -W -f '${package} ${status}\n'"
        cmd, cmds = _which_split_cmd(cmd)
        self.tue_install_echo(repr(cmd))
        sub = BackgroundPopen(
            args=cmds,
            out_handler=_out_handler_installed_pkgs,  # Needed to prevent buffer to get full
            err_handler=self._err_handler,
            stdin=sp.PIPE,
            text=True,
        )
        sub.wait()
        if sub.returncode != 0:
            self.tue_install_error(f"Error while getting installed packages({sub.returncode}):\n    {repr(cmd)}")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        installed_pkgs = [pkg[:-21] for pkg in installed_pkgs if pkg[-20:] == "install ok installed"]
        pkgs_to_install = []
        for pkg in pkgs:
            if pkg not in installed_pkgs:
                self.tue_install_debug(f"Package {pkg} is not installed")
                pkgs_to_install.append(pkg)
            else:
                self.tue_install_debug(f"Package {pkg} is already installed")

        if not pkgs_to_install:
            return True

        # Install packages
        apt_get_cmd = f"sudo apt-get install --assume-yes -q {' '.join(pkgs_to_install)}"
        self.tue_install_echo(f"Going to run the following command:\n{apt_get_cmd}")

        _wait_for_dpkg_lock()

        if not os.path.isfile(self._apt_get_updated_file):
            cmd = "sudo apt-get update"
            sub = self._default_background_popen(cmd)
            if sub.returncode != 0:
                self.tue_install_error(f"Error while updating apt-get({sub.returncode}):\n    {repr(cmd)}")
                # ToDo: This depends on behaviour of tue-install-error
                return False
            Path(self._apt_get_updated_file).touch()

        sub = self._default_background_popen(apt_get_cmd)
        if sub.returncode != 0:
            self.tue_install_error(
                f"Error while installing system packages({sub.returncode}):" f"\n    {repr(apt_get_cmd)}"
            )
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self.tue_install_debug(f"Installed {pkgs} ({sub.returncode})")

        return True

    def tue_install_apt_get_update(self):
        self.tue_install_debug("tue-install-apt-get-update")
        self.tue_install_debug("Requiring an update of apt-get before next 'apt-get install'")
        if os.path.isfile(self._apt_get_updated_file):
            os.remove(self._apt_get_updated_file)

    def tue_install_ppa(self, ppas: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-ppa {ppas=}")
        if not ppas:
            self.tue_install_error("Invalid tue-install-ppa call: needs ppas as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self._ppas.extend(ppas)
        return True

    def tue_install_ppa_now(self, ppas: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-ppa-now {ppas=}")

        if not ppas:
            self.tue_install_error("Invalid tue-install-ppa-now call: needs ppas as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        ppa_added = False
        for ppa in ppas:
            if not ppa.startswith("ppa:") and not ppa.startswith("deb "):
                self.tue_install_error(f"Invalid ppa: needs to start with 'ppa:' or 'deb ' ({ppa=})")
                # ToDo: This depends on behaviour of tue-install-error
                return False

            if ppa.startswith("ppa:"):
                ppa_pattern = f"^deb.*{ppa[4:]}".replace("[", "\[").replace("]", "\]").replace("/", "\/")
                ppa_pattern = re.compile(ppa_pattern)
                if grep_directory(self._sources_list_dir, ppa_pattern):
                    self.tue_install_debug(f"PPA '{ppa}' is already added previously")
                    continue

            elif ppa.startswith("deb "):
                ppa_pattern = f"^{ppa}$".replace("[", "\[").replace("]", "\]").replace("/", "\/")
                ppa_pattern = re.compile(ppa_pattern)
                if grep_file(self._sources_list, ppa_pattern):
                    self.tue_install_debug(f"PPA '{ppa}' is already added previously")
                    continue

            self.tue_install_system_now(["software-properties-common"])

            self.tue_install_info(f"Adding ppa: {ppa}")

            # Wait for apt-lock first (https://askubuntu.com/a/375031)
            _wait_for_dpkg_lock()

            cmd = f"sudo add-apt-repository --no-update --yes '{ppa}'"
            sub = self._default_background_popen(cmd)
            if sub.returncode != 0:
                self.tue_install_error(f"An error occurred while adding ppa: {ppa}")
                # ToDo: This depends on behaviour of tue-install-error
                return False

            ppa_added = True

        if ppa_added:
            self.tue_install_apt_get_update()

        return True

    def tue_install_pip(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-pip {pkgs=}")
        if not pkgs:
            self.tue_install_error("Invalid tue-install-pip call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self._pips.extend(pkgs)
        return True

    def tue_install_pip_now(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-pip-now {pkgs=}")

        if not pkgs:
            self.tue_install_error("Invalid tue-install-pip-now call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        pips_to_install = []
        git_pips_to_install = []
        for pkg in pkgs:
            if pkg.startswith("git+"):
                git_pips_to_install.append(pkg)
                continue

            req = pip_install_req_from_line(pkg)
            req.check_if_exists(use_user_site=True)

            if req.satisfied_by:
                self.tue_install_debug(f"{pkg} is already installed, {req.satisfied_by}")
            else:
                pips_to_install.append(pkg)

        if pips_to_install:
            returncode = pip_main(["install", "--user", *pips_to_install])
            if returncode != 0:
                self.tue_install_error(f"An error occurred while installing pip packages: {pips_to_install}")
                # ToDo: This depends on behaviour of tue-install-error
                return False

        if git_pips_to_install:
            for pkg in git_pips_to_install:
                returncode = pip_main(["install", "--user", pkg])
                if returncode != 0:
                    self.tue_install_error(f"An error occurred while installing pip packages: {pkg}")
                    # ToDo: This depends on behaviour of tue-install-error
                    return False

        return True

    def tue_install_snap(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-snap {pkgs=}")
        if not pkgs:
            self.tue_install_error("Invalid tue-install-snap call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self._snaps.extend(pkgs)
        return True

    def tue_install_snap_now(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-snap-now {pkgs=}")

        if not pkgs:
            self.tue_install_error("Invalid tue-install-snap-now call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self.tue_install_system_now(["snapd"])

        cmd = "snap list"
        cmd, cmds = _which_split_cmd(cmd)
        self.tue_install_echo(repr(cmd))
        sub = BackgroundPopen(args=cmds, err_handler=self._err_handler, stdout=sp.PIPE, text=True)
        sub.wait()
        if sub.returncode != 0:
            self.tue_install_error(f"Error while getting snap list({sub.returncode}):\n    {repr(cmd)}")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        snaps_installed = sub.stdout.read().splitlines()[1:]
        snaps_installed = [snap.split()[0] for snap in snaps_installed]
        snaps_to_install = []
        for pkg in pkgs:
            if pkg not in snaps_installed:
                snaps_to_install.append(pkg)
                self.tue_install_debug(f"Snap '{pkg}' is not installed yet")
            else:
                self.tue_install_debug(f"Snap '{pkg}' is already installed")

        if not snaps_to_install:
            self.tue_install_debug("No snaps to install")
            return True

        for pkg in snaps_to_install:
            cmd = f"sudo snap install --classic {pkg}"
            sub = self._default_background_popen(cmd)
            breakpoint()
            if sub.returncode != 0:
                self.tue_install_error(f"An error occurred while installing snap: {pkg}")
                # ToDo: This depends on behaviour of tue-install-error
                return False

        return True

    def tue_install_gem(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-gem {pkgs=}")
        if not pkgs:
            self.tue_install_error("Invalid tue-install-gem call: needs packages as argument")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self._gems.extend(pkgs)
        return True

    def tue_install_gem_now(self, pkgs: List[str]) -> bool:
        self.tue_install_debug(f"tue-install-gem-now {pkgs=}")

        if not pkgs:
            self.tue_install_error("Invalid tue-install-gem-now call: got an empty list of packages as argument.")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        self.tue_install_system_now(["ruby", "ruby-dev", "rubygems-integration"])

        cmd = "gem list"
        cmd, cmds = _which_split_cmd(cmd)
        self.tue_install_echo(repr(cmd))
        sub = BackgroundPopen(args=cmds, err_handler=self._err_handler, stdout=sp.PIPE, text=True)
        sub.wait()
        if sub.returncode != 0:
            self.tue_install_error(f"Error while getting installed gem packages({sub.returncode}):\n    {repr(cmd)}")
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
            sub = self._default_background_popen(cmd)
            if sub.returncode != 0:
                self.tue_install_error(
                    f"An error occurred while installing gem packages({sub.returncode}):\n    {repr(cmd)}"
                )
                # ToDo: This depends on behaviour of tue-install-error
                return False

        return True

    def tue_install_dpkg_now(self, pkg_file: str) -> bool:
        self.tue_install_debug(f"tue-install-dpkg-now {pkg_file=}")

        if not pkg_file:
            self.tue_install_error("Invalid tue-install-dpkg-now call: got an empty package file as argument.")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        if not os.path.isabs(pkg_file):
            pkg_file = os.path.join(self._current_target_dir, pkg_file)

        if not os.path.isfile(pkg_file):
            self.tue_install_error(f"Invalid tue-install-dpkg-now call: {pkg_file} is not a file.")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        cmd = f"sudo dpkg -i {pkg_file}"
        sub = self._default_background_popen(cmd)
        # ToDo: Should we check the return code?
        # if sub.returncode != 0:
        #     self.tue_install_error(f"An error occurred while installing dpkg package({sub.returncode}):"
        #                            f"\n    {repr(cmd)}")
        #     # ToDo: This depends on behaviour of tue-install-error
        #     return False

        cmd = f"sudo apt-get install -f"
        sub = self._default_background_popen(cmd)
        if sub.returncode != 0:
            self.tue_install_error(f"An error occurred while fixing dpkg isntall({sub.returncode}):\n    {repr(cmd)}")
            # ToDo: This depends on behaviour of tue-install-error
            return False

        return True

    tue_install_dpkg = tue_install_dpkg_now

    def tue_install_ros(self, source_type: str, **kwargs) -> bool:
        return True


if __name__ == "__main__":
    bla = InstallerImpl(debug=True)
    bla.tue_install_target("networking")
