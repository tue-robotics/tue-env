#!/usr/bin/env python
import getpass
import paramiko
import socket
import signal
import subprocess
import sys
from time import sleep
from os import getuid, listdir, devnull

# Lets start with catching Ctrl+C's
def signal_handler(signal, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Questions will need to be asked:
def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    Source: https://stackoverflow.com/questions/3041986/apt-command-line-interface-like-yes-no-input
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")

# Create a /dev/null to suppress output:
FNULL = open(devnull, 'w')

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
localuser = getpass.getuser()
if user == '':
    user = localuser

password=''

# Check for existance of ssh keys
if not "id_rsa" in listdir("/home/%s/.ssh" % (localuser)):
    print("ssh keys are missing, lets generate some")
    command = "cat /dev/zero | ssh-keygen -q -N "
    subprocess.call(command, shell=True)

print('ssh %s@%s' % (user, host))

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#agent = paramiko.Agent()

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
        if subprocess.call('ssh-keygen -F %s' % host, shell=True, stdout=FNULL, stderr=subprocess.STDOUT) == 0:
            command = 'ssh-keygen -R %s' % host
            if query_yes_no('A keyfile was found however. Do you like to remove the keyfile? (%s)' % command):
                subprocess.call(command, shell=True)
            else:
                sys.exit(2) #Suit yourself then

        command = 'ssh-copy-id %s@%s' % (user, host)
        if query_yes_no('Do you want to store your credentials? (%s)' % command):
            password = getpass.getpass('Password for %s@%s: ' % (user, host))
            subprocess.call(command, shell=True)
        else:
            sys.exit(0) #This allows the user to continue to the actual ssh command (specified after this script in an alias)
    except Exception, e:
        print e
        raise
        print("Unknown error, please report an issue here: https://github.com/tue-robotics/tue-env/issues/new")
        sys.exit(2)
