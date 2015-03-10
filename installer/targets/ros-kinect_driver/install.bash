if [ ! -d /usr/local/include/libfreenect ] && [ ! -f /usr/include/libfreenect.h ]
then
    sudo add-apt-repository ppa:floe/libtisch
    sudo apt-get update
    sudo apt-get install libfreenect libfreenect-dev libfreenect-demos
fi

