#!/bin/bash

# copying shortcuts to desktop
echo copying shortcuts to desktop, please drag them into the top bar
echo to do this, open a nautilus, go to the deskop, show hidden files and drag the hidden files to the top bar, do not delete the hidden files afterwards
cp ~/.tue/installer/targets/tue-shortcuts/shortcuts/.*.desktop ~/Desktop

# copying shortcuts to desktop
echo copying icons to /usr/share/pixmaps/tue
sudo mkdir /usr/share/pixmaps/tue
sudo cp ~/.tue/installer/targets/tue-shortcuts/icons/*.png /usr/share/pixmaps/tue/

# copying terminator config file
read -r -p "Are you sure you want to replace .config/terminator/config? [y/N] " response
response=${response,,}    # tolower
if [[ $response =~ ^(yes|y)$ ]]
then
	cp ~/.config/terminator/config ~/.config/terminator/configbackup
    cp ~/.tue/installer/targets/tue-shortcuts/terminatorconfig/config ~/.config/terminator/config
    echo the config has been copied. Backup of old config: ~/.config/terminator/configbackup
else
	echo the config has not been copied, Therefore the working of the shortcuts cannot be guaranteed
fi
