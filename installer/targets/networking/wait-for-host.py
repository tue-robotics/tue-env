#!/usr/bin/env python
import getpass
import paramiko
import socket
import signal
import subprocess
import sys
from time import sleep
from os import getuid, listdir

# Lets start with catching Ctrl+C's
def signal_handler(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Host name should be given as an argument
if len(sys.argv) < 2:
    print "No host specified, Usage example: wait-for-host.py localhost"
    sys.exit(1)

# get hostname
user = ''
if len(sys.argv) > 1:
    host = sys.argv[1]
    if host.find('@') >= 0:
        user, host = host.split('@')
else:
    host = input('Hostname: ')
if len(host) == 0:
    print('*** Hostname required.')
    sys.exit(1)
    
# get user
if user == '':
    user = getpass.getuser()
    
password=''

# Check for existance of ssh keys
PRIV_SSH_DIR = "/home/%s/.ssh" % (user)
if not "id_rsa" in listdir(PRIV_SSH_DIR):
    print("ssh keys are missing, lets generate some")
    command = "cat /dev/zero | ssh-keygen -q -N "
    subprocess.call(command, shell=True)

print('ssh %s@%s' % (user, host))

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

success = False
while not success:
    try:
        ssh.connect(host,22,user,password,allow_agent=False)
        success = True
        ssh.close()
    except (socket.gaierror):
        # Host not found
        print('Host %s not (yet) found, retrying...' % host)
        sleep(1)
    except(paramiko.ssh_exception.AuthenticationException):
        print('You are not authorised!')
        password = getpass.getpass('Password for %s@%s: ' % (user, host))
        print('Do you want to store your credentials?')
        choice = raw_input().lower()
        if choice in {'yes','y', 'ye', ''}:
            command = 'ssh-copy-id %s@%s' % (user, host)
            subprocess.call(command, shell=True)
    except Exception, e:
        print e
        raise
        print("Unknown error, please report an issue here: https://github.com/tue-robotics/tue-env/issues/new")
        sys.exit(2)
