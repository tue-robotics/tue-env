if [ ! $(pip freeze | grep docopt) ]
then
	sudo pip install -U docopt
fi
