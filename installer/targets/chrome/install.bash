#!/bin/bash

if [ ! -f /etc/apt/sources.list.d/google-chrome.list ]
then
    wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
    sudo sh -c 'echo "deb http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
    sudo apt-get update
fi

tue-install-system google-chrome-stable
