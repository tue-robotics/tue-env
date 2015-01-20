if [ ! $(pip freeze | grep xhtml2pdf) ]
then
	sudo pip install -U xhtml2pdf
fi
