if [ ! -f /etc/apt/sources.list.d/gazebo-latest.list ]
then
    # Setup your computer to accept software from packages.osrfoundation.org.
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'

    # Retrieve and install the keys for the Gazebo repositories.
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

    sudo apt-get update
fi

# install Gazebo
tue-install-system gazebo
