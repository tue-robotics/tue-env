if [ ! -f /etc/apt/sources.list.d/docker.list ]
then
	wget https://get.docker.io/ubuntu/ -O /tmp/docker-install && sudo sh /tmp/docker-install
fi
