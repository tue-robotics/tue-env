if [ ! -d /usr/local/include/libfreenect ] && [ ! -f /usr/include/libfreenect.h ]
then
    sudo apt-get install --assume-yes libfreenect-dev
fi

