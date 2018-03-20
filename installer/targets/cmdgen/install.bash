#!/bin/bash

if [ ! -d ~/src/GPSRCmdGen ]
then
    tue-install-debug "GPSRCmdGen not yet installed"
    echo "Installing GPSRCmdGen to ~/src/GPSRCmdGen"

    echo "Installing dependency of GPSRCmdGen: mono-complete"
    sudo apt-get install mono-complete

    tue-install-git https://github.com/tue-robotics/GPSRCmdGen.git ~/src/GPSRCmdGen

    echo "Making GPSRCmdGen"
    cd ~/src/GPSRCmdGen
    make

else # GPSRCmdGen installed
    cd ~/src/GPSRCmdGen
    REMOTE=$(git config --get remote.origin.url)
    if [ "$REMOTE" != "https://github.com/tue-robotics/GPSRCmdGen.git" ]; then
        echo "The GPSRCmdGen is still pointing to old remote, will be changed to tue-fork"
        git remote set-url origin https://github.com/tue-robotics/GPSRCmdGen.git
    else
      echo "The GPSRCmdGen is correctly pointing to tue-fork"
    fi
    echo "Pulling GPSRCmdGen"
    prev=$(git rev-list HEAD -n 1)
    git pull origin master

    # https://stackoverflow.com/questions/20995026/how-to-determine-whether-a-git-pull-did-something-from-a-shell-script
    if [ "$prev" != "$(git rev-list HEAD -n 1)" ]; then
      echo "Making GPSRCmdGen"
      make
    else
      echo "Build already up-to-date"
    fi
fi
