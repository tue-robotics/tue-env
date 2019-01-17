#!/bin/bash
source /etc/lsb-release

# copying shortcuts to desktop
tue-install-cp shortcuts/.*.desktop ~/Desktop
if [ ! -f ~/.local/share/applications/AMIGO\ Dashboard.desktop ]
then
    tue-install-cp shortcuts/.*.desktop ~/.local/share/applications/
    find ~/.local/share/applications/ -iname ".*.desktop" -exec rename 's/\.Terminator/Terminator/' '{}' \;
    find ~/.local/share/applications/ -iname ".*.desktop" -exec rename 's/\.AMIGO/AMIGO/' '{}' \;
    find ~/.local/share/applications/ -iname ".*.desktop" -exec rename 's/\.SERGIO/SERGIO/' '{}' \;
fi

# copying shortcuts to desktop
if [ ! -d /usr/share/pixmaps/tue ]
then
    sudo mkdir -p /usr/share/pixmaps/tue
    tue-install-cp icons/*.png /usr/share/pixmaps/tue/
fi

# copying terminator config file
mkdir -p ~/.config/terminator
[ -f ~/.config/terminator/config ] && cp ~/.config/terminator/config ~/.config/terminator/configbackup
tue-install-cp configs/terminator/config ~/.config/terminator/config


# copying variety config file
if [ -f ~/.config/variety/variety.conf ]; then
    cp ~/.config/variety/variety.conf ~/.config/variety/variety.conf.backup
else
    mkdir -p ~/.config/variety/
fi
tue-install-cp configs/variety/variety.conf ~/.config/variety/variety.conf
