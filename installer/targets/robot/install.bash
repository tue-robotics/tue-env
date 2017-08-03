#Do not require sudo
if [ ! -f /etc/sudoers.d/sudo-no-password ]
then
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' | sudo tee /etc/sudoers.d/sudo-no-password && \
    sudo chmod 440 /etc/sudoers.d/sudo-no-password
fi

### Make sure remote launching works ###
tue-install-cp ros-bash-and-run.sh ~/.ros-bash-and-run.sh
chmod +x ~/.ros-bash-and-run.sh

### Allow user to access serial interfaces ###
if ! groups ${USER} | grep -q dialout
  then
    sudo gpasswd --add ${USER} dialout
fi
