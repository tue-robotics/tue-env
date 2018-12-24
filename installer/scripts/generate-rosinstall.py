#!/usr/bin/env python

import os
import vcstools
import yaml
import sys

if len(sys.argv) < 2:
    rootDir = '.'
else:
    rootDir = sys.argv[1]    

def find_source_control_dirs(directory, level=2):
    directory = directory.rstrip(os.path.sep)
    assert os.path.isdir(directory)
    for root, dirs, files in os.walk(directory):

        vcs_client = get_vcs_client(root)
        if vcs_client != None:
            del dirs[:]
            yield root, vcs_client

def get_vcs_client(directory):
    svn = vcstools.SvnClient(directory)
    if svn.detect_presence():
        return svn
    git = vcstools.GitClient(directory)
    if git.detect_presence():
        return git

vcss = []
for dirName, vcs_client in find_source_control_dirs(rootDir):
    vcss.append({
        vcs_client.get_vcs_type_name().encode('ascii'): {
            'local-name': dirName,
            'uri': vcs_client.get_url().encode('ascii')
        }
    })

print yaml.dump(vcss)
