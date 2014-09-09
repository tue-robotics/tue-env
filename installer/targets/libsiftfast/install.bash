if [ ! -f /usr/local/lib/libsiftfast.so ]
then
    svn co http://svn.code.sf.net/p/libsift/code/trunk /tmp/libsiftfast
    mkdir /tmp/libsiftfast/build
    cd /tmp/libsiftfast/build

    #cd /tmp
    #wget http://downloads.sourceforge.net/project/libsift/libsiftfast/libsiftfast-1.2/libsiftfast-1.2-src.tgz
    #tar -xzvf libsiftfast-1.2-src.tgz
    #mkdir libsiftfast-1.2-src/build
    #cd libsiftfast-1.2-src/build

    cmake ..
    make
    sudo make install
fi
