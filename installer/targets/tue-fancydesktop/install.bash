#!/bin/bash
source /etc/lsb-release

# copying shortcuts to desktop
cp $TUE_DIR/installer/targets/tue-fancydesktop/shortcuts/.*.desktop ~/Desktop

# copying shortcuts to desktop
if [ ! -d /usr/share/pixmaps/tue ]
then
    sudo mkdir -p /usr/share/pixmaps/tue
    sudo cp $TUE_DIR/installer/targets/tue-fancydesktop/icons/*.png /usr/share/pixmaps/tue/
fi

# copying terminator config file
mkdir -p ~/.config/terminator
[ -f ~/.config/terminator/config ] && cp ~/.config/terminator/config ~/.config/terminator/configbackup
cp $TUE_DIR/installer/targets/tue-fancydesktop/configs/terminator/config ~/.config/terminator/config

# Install Variety
if ! grep -q -r --include \*.list "peterlevi" /etc/apt 
then
	sudo add-apt-repository ppa:peterlevi/ppa
	sudo apt-get update
    sudo apt-get install -y variety
fi

# Secondly fix variety config file
cp ~/.config/variety/variety.conf ~/.config/variety/variety.conf.backup
cp $TUE_DIR/installer/targets/tue-fancydesktop/configs/variety/variety.conf ~/.config/variety/variety.conf
