import os
from appdirs import AppDirs

PACKAGE_NAME = __package__.split(".")[0]
USER = os.environ["USER"]
PACKAGE_APPDIRS = AppDirs(PACKAGE_NAME, USER)

PACKAGE_CONFIG_DIR = PACKAGE_APPDIRS.user_config_dir
WS_FILE_DIR = os.path.join(PACKAGE_CONFIG_DIR, "ws")
