## Chrony 
# If config file does not exist, chrony is probably not installed 
if [ ! -f /etc/chrony/chrony.conf ]
then
    echo "I guess chrony is not installed"
    sudo apt-get install --assume-yes chrony
fi

# If clephas (the author) is not in the config, it's probably not the correct one
# Hence: copy
if ! cmp /etc/chrony/chrony.conf ~/.tue/installer/targets/sergio1/chrony.conf --quiet
then
    echo "Chrony config is probably not correct, will copy"
    
    # Backup old config
    sudo mv /etc/chrony/chrony.conf /etc/chrony/chrony.conf.backup
    
    # Copy new config
    sudo cp ~/.tue/installer/targets/sergio1/chrony.conf /etc/chrony/chrony.conf
    
    # Restart chrony
    sudo service chrony restart
fi


