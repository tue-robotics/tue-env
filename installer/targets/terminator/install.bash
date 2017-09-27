if [ ! -d ~/.config/terminator ]
then
    tue-install-debug "creating ~/.config/terminator, because not existing yet"
    mkdir -p ~/.config/terminator
else
    tue-install-debug "~/.config/terminator already exists"
fi

if [ -f ~/.config/terminator/config ]
then
    tue-install-debug "tue-tue-install-cp config ~/.config/terminator/config2"
    tue-install-cp config ~/.config/terminator/config2
else
    tue-install-debug "tue-tue-install-cp config ~/.config/terminator/config"
    tue-install-cp config ~/.config/terminator/config
fi
