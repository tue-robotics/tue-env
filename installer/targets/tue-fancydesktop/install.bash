#!/bin/bash
source /etc/lsb-release

# copying shortcuts to desktop
cp ~/.tue/installer/targets/tue-fancydesktop/shortcuts/.*.desktop ~/Desktop

# copying shortcuts to desktop
sudo mkdir -p /usr/share/pixmaps/tue
sudo cp ~/.tue/installer/targets/tue-fancydesktop/icons/*.png /usr/share/pixmaps/tue/

# copying terminator config file
cp ~/.config/terminator/config ~/.config/terminator/configbackup
mkdir -p ~/.config/terminator/config
cp ~/.tue/installer/targets/tue-fancydesktop/configs/terminator/config ~/.config/terminator/config

# Install Variety
if ! grep -q -r --include \*.list "peterlevi" /etc/apt 
then
	sudo add-apt-repository ppa:peterlevi/ppa
	sudo apt-get update
fi
sudo apt-get install -y variety

# Secondly fix variety config file
cp ~/.config/variety/variety.conf ~/.config/variety/variety.conf.backup
cp ~/.tue/installer/targets/tue-fancydesktop/configs/variety/variety.conf ~/.config/variety/variety.conf
