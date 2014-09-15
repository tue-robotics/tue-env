if [ ! -f /etc/apt/sources.list.d/spotify.list ]
then
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 94558F59
    sudo sh -c 'echo "deb http://repository.spotify.com stable non-free" >> /etc/apt/sources.list.d/spotify.list'

    sudo apt-get update
fi

tue-install-system spotify-client
