if dpkg -s libmedia1 | grep 'Architecture: amd64' &> /dev/null
then
    sudo apt remove libmedia1 -y
fi
