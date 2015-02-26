if [ ! -d /usr/local/include/libfreenect ]
then
    sudo add-apt-repository ppa:floe/libtisch
    sudo apt-get update
    sudo apt-get install libfreenect libfreenect-dev libfreenect-demos
fi

