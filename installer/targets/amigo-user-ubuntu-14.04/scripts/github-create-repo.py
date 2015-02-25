#!/usr/bin/env python

# pip install PyGithub

from github import Github
import getpass, sys

# argument parsing
args = sys.argv
if len(args) != 2:
	print 'usage: %s <repo name>' % args[0]
	exit()
repo_name = args[1]

# prompt for username + password
def login():
    user = raw_input("Username [%s]: " % getpass.getuser())
    if not user:
        user = getpass.getuser()
    return user, getpass.getpass()

user, password = login()

g = Github(user, password)
org = g.get_organization('tue-robotics')

print "I'm going to create a repository named '%s' in '%s'" % (repo_name, org.name)
org.create_repo(name=repo_name)