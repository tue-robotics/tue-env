#! /usr/bin/env python3
import importlib.resources as resources
import os
import shutil
import tempfile
import subprocess
import datetime
from ..utils import PACKAGE_NAME, PACKAGE_CONFIG_DIR, WS_FILE_DIR

DEFAULT_WS_CONFIG_FILE = os.path.join(PACKAGE_CONFIG_DIR, "default_ws")
CURRENT_WS_CONFIG_FILE = os.path.join(tempfile.gettempdir(), PACKAGE_NAME, "current_env")


def get_setup_path(verbose) -> None:
    with resources.path(PACKAGE_NAME, "setup.bash") as setup_file_path:
        print(setup_file_path)


def get_workspaces():
    try:
        return os.listdir(WS_FILE_DIR)
    except Exception:
        return []


def init(verbose, name, dir, targets_git_url):
    ws_file = os.path.join(WS_FILE_DIR, name)
    ws_dir = os.path.abspath(os.path.join(os.getcwd(), dir))
    ws_env_dir = os.path.join(ws_dir, ".env")
    ws_setup = os.path.join(ws_env_dir, "setup", "user_setup.bash")

    if os.path.isfile(ws_file):
        raise Exception(f"[{PACKAGE_NAME}] Workspace '{name}' already exists")

    if os.path.isdir(ws_env_dir):
        raise Exception(f"[{PACKAGE_NAME}] Directory '{dir}' is already a workspace directory")

    os.makedirs(WS_FILE_DIR, exist_ok=True)
    with open(ws_file, "w") as file:
        file.write(f"{ws_dir}")

    os.makedirs(os.path.dirname(ws_setup))
    with open(ws_setup, "w") as file:
        file.write("#! /usr/bin/env bash\n")

    print(f"[{PACKAGE_NAME}] Created new workspace {name}")

    if targets_git_url:
        init_targets(verbose, targets_git_url, name)


def set_default(verbose, name):
    if not name:
        name = "\n"

    os.makedirs(os.path.dirname(DEFAULT_WS_CONFIG_FILE), exist_ok=True)

    with open(DEFAULT_WS_CONFIG_FILE, "w") as file:
        file.write(f"{name}")


def _get_default():
    if not os.path.isfile(DEFAULT_WS_CONFIG_FILE):
        return ""

    with open(DEFAULT_WS_CONFIG_FILE, "r") as file:
        default_ws = file.readlines()
        if len(default_ws) != 1:
            raise Exception(f"Default WS config file '{DEFAULT_WS_CONFIG_FILE}' corrupted. Please delete it first.")

        default_ws = default_ws[0]
        default_ws = default_ws.strip()
        default_ws = default_ws.rstrip()

        if default_ws:
            if default_ws not in get_workspaces():
                raise Exception(f"Default WS config file '{DEFAULT_WS_CONFIG_FILE}' corrupted. Please delete it first.")

        return default_ws


def get_default(verbose):
    print(_get_default())


def list_workspaces(verbose=None):

    ws = get_workspaces()
    print(" ".join(ws))


def _locate(ws=None):
    if not ws:
        ws = _get_current()

    if not ws:
        return ""

    with open(os.path.join(WS_FILE_DIR, ws), "r") as file:
        ws_dir = file.readlines()[0]
        ws_dir = ws_dir.strip()
        ws_dir = ws_dir.rstrip()

        if not os.path.isdir(os.path.join(ws_dir, ".env")):
            raise Exception(f"Worspace directory '{ws_dir}' does not exist")

        return ws_dir


def locate(verbose):
    print(_locate())


def _get_current():
    if not os.path.isfile(CURRENT_WS_CONFIG_FILE):
        return _get_default()

    with open(CURRENT_WS_CONFIG_FILE, "r") as file:
        current_ws = file.readlines()
        if len(current_ws) != 1:
            raise Exception(f"Current WS config file '{CURRENT_WS_CONFIG_FILE}' corrupted. Please delete it first.")

        current_ws = current_ws[0]
        current_ws = current_ws.strip()
        current_ws = current_ws.rstrip()

        if current_ws:
            if current_ws not in get_workspaces():
                raise Exception(f"Current WS config file '{CURRENT_WS_CONFIG_FILE}' corrupted. Please delete it first.")

        return current_ws


def get_current(verbose):
    print(_get_current())


def switch_workspaces(verbose, name):
    default_ws = _get_default()

    if name == default_ws:
        if os.path.isfile(CURRENT_WS_CONFIG_FILE):
            os.remove(CURRENT_WS_CONFIG_FILE)

    else:
        os.makedirs(os.path.dirname(CURRENT_WS_CONFIG_FILE), exist_ok=True)

        with open(CURRENT_WS_CONFIG_FILE, "w") as file:
            file.write(f"{name}")

    print(f"Workspace switched to {name}. Open a new terminal to make it effective.")


def init_targets(verbose, url, workspace):
    ws_dir = _locate(workspace)

    targets_dir = os.path.join(ws_dir, ".env", "targets")
    if os.path.isdir(targets_dir):
        timestamp = str(datetime.datetime.now()).replace("-", "")
        timestamp = timestamp.replace(":", "")
        timestamp = timestamp.replace(".", "")
        timestamp = timestamp.replace(" ", "")
        shutil.move(targets_dir, targets_dir + "." + timestamp)

    retcode = subprocess.run(["git", "clone", "--recursive", url, targets_dir])
    return retcode.returncode


def targets(verbose):
    ws_dir = _locate()

    if not ws_dir:
        return

    targets_dir = os.path.join(ws_dir, ".env", "targets")
    if os.path.isdir(targets_dir):
        print(targets_dir)


def remove(verbose, name, purge):
    timestamp = str(datetime.datetime.now()).replace("-", "")
    timestamp = timestamp.replace(":", "")
    timestamp = timestamp.replace(".", "")
    timestamp = timestamp.replace(" ", "")

    default_ws = _get_default()
    current_ws = _get_current()

    for ws in name:
        ws_dir = _locate(ws)

        if not purge:
            shutil.move(ws_dir, ws_dir + "." + timestamp)
        else:
            shutil.rmtree(ws_dir)

        os.remove(os.path.join(WS_FILE_DIR, ws))

        if ws == default_ws and os.path.isfile(DEFAULT_WS_CONFIG_FILE):
            os.remove(DEFAULT_WS_CONFIG_FILE)

        if ws == current_ws and os.path.isfile(CURRENT_WS_CONFIG_FILE):
            os.remove(CURRENT_WS_CONFIG_FILE)
