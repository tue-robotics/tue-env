import os
import site
from .utils.dirs import PACKAGE_NAME

site_packages_folder = site.getusersitepackages()

# assuming this current file is located in the root of your package
current_package_root = os.path.dirname(os.path.dirname(__file__))

installed_as_editable  = False

egg_link_file1 = os.path.join(site_packages_folder, PACKAGE_NAME + ".egg-link")
egg_link_file2 = os.path.join(site_packages_folder, PACKAGE_NAME.replace("_", "-") + ".egg-link")

if os.path.isfile(egg_link_file1):
    egg_link_file = egg_link_file1
else:
    egg_link_file = egg_link_file2

if os.path.isfile(egg_link_file):
    with open(egg_link_file, "r") as file:
        linked_folder = file.readline()
        installed_as_editable = current_package_root in linked_folder

print(installed_as_editable)
