if dpkg -s libmedia1 | grep 'Architecture: amd64' &> /dev/null
then
    echo "Removing wrong version of libmedial"
    sudo apt remove libmedia1 -y
fi
