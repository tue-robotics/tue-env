#!/bin/bash
source /etc/lsb-release

# copying shortcuts to desktop
cp ~/.tue/installer/targets/tue-shortcuts/shortcuts/.*.desktop ~/Desktop

# copying shortcuts to desktop
sudo mkdir -p /usr/share/pixmaps/tue
sudo cp ~/.tue/installer/targets/tue-shortcuts/icons/*.png /usr/share/pixmaps/tue/

# copying terminator config file
read -r -p "Are you sure you want to replace .config/terminator/config? [y/N] " response
response=${response,,}    # tolower
if [[ $response =~ ^(yes|y)$ ]]
then
	cp ~/.config/terminator/config ~/.config/terminator/configbackup
    cp ~/.tue/installer/targets/tue-shortcuts/configs/terminator/config ~/.config/terminator/config
else
	echo the terminator config has not been copied, Therefore the working of the shortcuts cannot be guaranteed
fi

# install variety
read -r -p "Do you want to install variety to show fancy amigo pictures on your desktop? [y/N] " response
response=${response,,}    # tolower
if [[ $response =~ ^(yes|y)$ ]]
then
	# First install variety
	if ! grep -q -r --include \*.list "peterlevi" /etc/apt 
	then
		sudo add-apt-repository ppa:peterlevi/ppa
		sudo apt-get update
	fi
	tue-install-system variety
	
	# Secondly fix variety config file
	read -r -p "Are you sure you want to replace .config/variety/config? [y/N] " response
	response=${response,,}    # tolower
	if [[ $response =~ ^(yes|y)$ ]]
	then
		cp ~/.config/variety/variety.conf ~/.config/variety/variety.conf.backup
		cp ~/.tue/installer/targets/tue-shortcuts/configs/variety/variety.conf ~/.config/variety/variety.conf
	else
		echo the variety config has not been copied, Therefore the working of the shortcuts cannot be guaranteed
	fi
fi

echo 
echo Finished. Please drag the hidden desktopicons from the desktop into the top bar
