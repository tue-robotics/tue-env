#!/bin/bash

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
    cp ~/.tue/installer/targets/tue-shortcuts/terminatorconfig/config ~/.config/terminator/config
else
	echo the config has not been copied, Therefore the working of the shortcuts cannot be guaranteed
fi

echo 
echo Finished. Please drag the hidden desktopicons from the desktop into the top bar
