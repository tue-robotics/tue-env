## Chrony
set -e

# If config file does not exist, chrony is probably not installed 
if [ ! -f /etc/chrony/chrony.conf ]
then
    echo "I guess chrony is not installed"
    sudo apt-get install --assume-yes chrony
fi

# Check config file
if ! cmp /etc/chrony/chrony.conf $TUE_DIR/installer/targets/amigo-chrony-client-pc/chrony.conf --quiet
then
    echo "Chrony config is probably not correct, will copy"
    
    # Copy and backup old config
    sudo install --backup=numbered --compare --verbose $TUE_DIR/installer/targets/amigo-chrony-client-pc/chrony.conf /etc/chrony/chrony.conf
    
    # Restart chrony
    sudo service chrony restart
fi
