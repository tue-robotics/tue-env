#! /usr/bin/python

"""
Usage: tue_installer.py YAML_FILE TARGET OUTPUT_FILE
"""

import sys
import yaml

class Target:

    def __init__(self):
        self.deps = []

class InstallData:

    def __init__(self):
        self.system_deps = []
        self.svn_deps = []
        self.install_options = {'system': self.insert_system_dep,
                                'svn'   : self.insert_svn_dep}

    def insert(self, install_type, value):
        try:
            self.install_options[install_type](value)
        except KeyError:
            print "Unknown install type: '{0}'".format(install_type)

    def insert_system_dep(self, dep):
        self.system_deps += [dep]

    def insert_svn_dep(self, dep):
        url = dep['url']
        self.svn_deps += [url]

    def __repr__(self):
        return self.generateBashScript()

    def generateBashScript(self):
        s = ""
        if self.system_deps:
            s += "sudo apt-get install -y"
            for dep in self.system_deps:
                s += " " + str(dep)
            s += "\n"

        if self.svn_deps:            
            for dep in self.svn_deps:
                s += "svn co " + str(dep) + "\n"

        return s
                
        

def install(target, yaml_db, install_data):
    try:
        info = yaml_db[target]
    except KeyError:
        print "Could not find target {0}".format(target)
        return False

    try:
        target_type = info['type']
    except KeyError:
        pass

    try:
        source = info['source']
    except KeyError:
        pass

    deps = []
    try:
        deps = info['deps']
    except KeyError:
        pass

    for dep_type, dep_name in deps.iteritems():
        print "dep: {0}".format(dep)
        
    return True

def main():
    if len(sys.argv) != 3:
        print __doc__
        return 2

    yaml_db_name = sys.argv[1]
    target = sys.argv[2]

    try:
        f = open(yaml_db_name)    
        yaml_db = yaml.safe_load(f)
        f.close()
    except:
        print "Failed to open YAML DB '{0}'".format(yaml_db)
        return 2

    install_data = InstallData()

    install(target, yaml_db, install_data)

    print install_data

if __name__ == "__main__":
    sys.exit(main())
