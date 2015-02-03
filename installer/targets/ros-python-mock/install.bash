if [ ! $(pip freeze | grep mock) ]
then
	sudo pip install -U mock
fi
