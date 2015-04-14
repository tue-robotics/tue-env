if [ ! -d /usr/local/include/libfreenect ] && [ ! -f /usr/include/libfreenect.h ]
then
    if [ `lsb_release -rs` == "12.04" ]
    then
        sudo add-apt-repository ppa:floe/libtisch
        sudo apt-get update
        sudo apt-get install libfreenect libfreenect-dev libfreenect-demos
    else
        sudo apt-get install libfreenect-dev
    fi
fi

