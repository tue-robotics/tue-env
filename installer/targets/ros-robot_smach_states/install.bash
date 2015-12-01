# Make sure python-pip is installed
dpkg -s python-pip &> /dev/null || sudo apt-get install python-pip

sudo apt-get install -y ros-$TUE_ROS_DISTRO-orocos-kdl

if [ ! $(pip freeze | grep graphviz) ]
then
	sudo -H pip install -U graphviz
fi

if [ ! $(pip freeze | grep psutil) ]
then
	sudo -H pip install -U psutil
fi
