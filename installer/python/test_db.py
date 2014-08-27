#! /usr/bin/python

import tue_install.db

#print tue_install.db.all_targets()
print tue_install.db.get_install_deps("ros-ed")
