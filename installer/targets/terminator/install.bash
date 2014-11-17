if [ -d ~/.config/terminator ]
then

    if [ -f ~/.config/terminator/config ]
    then
	    tue-install-cp config ~/.config/terminator/config2
    else
        tue-install-cp config ~/.config/terminator/config
    fi
fi
