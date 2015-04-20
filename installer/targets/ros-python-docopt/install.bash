# Make sure python-pip is installed
dpkg -s python-pip &> /dev/null || sudo apt-get install python-pip

if ! pip freeze | grep docopt
then
	sudo -H pip install -U docopt
fi
