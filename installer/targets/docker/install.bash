if [ ! -f /etc/apt/sources.list.d/docker.list ]
then
	wget https://get.docker.com -O /tmp/docker-install && sudo sh /tmp/docker-install

	# Add the docker group if it doesn't already exist.
	sudo groupadd docker

	# Add the connected user "${USER}" to the docker group.
	# Change the user name to match your preferred user.
	# You may have to logout and log back in again for
	# this to take effect.
	sudo gpasswd -a ${USER} docker

	# Activate group permissions for this terminal
	newgrp docker

	# Restart the Docker daemon.
	# If you are in Ubuntu 14.04, use docker.io instead of docker
	sudo service docker restart
fi
