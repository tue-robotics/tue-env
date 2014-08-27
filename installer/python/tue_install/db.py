import os
import yaml

TARGETS_DIR = os.environ['HOME'] + "/.tue/installer/targets/"

def all_targets():
    if not os.path.isdir(TARGETS_DIR):
        return []
    else:
        return os.listdir(TARGETS_DIR)

# ----------------------------------------------------------------------------------------------------

def get_install_deps(target):
    target_file = TARGETS_DIR + target + "/install.yaml"

    install_items = []
    if os.path.exists(target_file):
        with open(target_file, 'r') as f:
            try:
                install_items = yaml.safe_load(f)
            except yaml.parser.ParserError:
                return []

    return install_items
