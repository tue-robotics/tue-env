if [ ! -f /usr/local/lib/libzmq.so.4 ]
then
	wget http://download.zeromq.org/zeromq-4.1.0-rc1.tar.gz -O /tmp/zeromq-4.1.0-rc1.tar.gz
	tar -zxvf /tmp/zeromq-4.1.0-rc1.tar.gz -C /tmp
	cd /tmp/zeromq-4.1.0
	./configure
	make
	sudo make install
	cd -
fi
