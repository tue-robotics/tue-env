# Get script dir
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create MEGA DIR is not exists
DATA_DIR="$HOME/MEGA"
if [ ! -d "$DATA_DIR" ]; then
    # Create data dir
    echo "Creating MEGA dir in home"
    mkdir $DATA_DIR
fi

# Setup mega rc file
if [ ! -f "$HOME/.megarc" ]; then
    # Create .megarc file in $HOME
    echo "Copying megarc file to home"
    cp $DIR/.megarc $HOME/.megarc

    echo "Please type the MEGA password of the amigoathome@gmail.com account"
    read -s -p password: passwd

    echo "Adding password to .megarc file"
    echo "Password = $passwd" >> $HOME/.megarc

    echo "Setting permissions to .megarc file"
    chmod 640 ~/.megarc
fi

# Setup the sync service
if [ ! -f "$HOME/.config/systemd/user/mega.timer" ]; then
    # Create mega service
    echo "Copying mega service files to ~/.config/systemd/user/"
    install -D $DIR/mega.service $HOME/.config/systemd/user/mega.service
    install -D $DIR/mega.timer $HOME/.config/systemd/user/mega.timer

    # Start the timer sync service
    systemctl --user start mega.timer

    # Auto enable on startup
    systemctl --user enable mega.timer
fi

