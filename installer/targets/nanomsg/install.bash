if [ ! -f /usr/local/lib/libnanomsg.so.0.2.1 ]
then
	wget http://download.nanomsg.org/nanomsg-0.5-beta.tar.gz -O /tmp/nanomsg-0.5-beta.tar.gz
	tar -zxvf /tmp/nanomsg-0.5-beta.tar.gz -C /tmp
	cd /tmp/nanomsg-0.5-beta
	./configure
	make
	make check && sudo make install
	cd -
fi
