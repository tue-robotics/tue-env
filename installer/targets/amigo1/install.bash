## Chrony 
# If config file does not exist, chrony is probably not installed 
if [ ! -f /etc/chrony/chrony.conf ]
then
    echo "I guess chrony is not installed"
    tue-install-system-now chrony
fi

# If clephas (the author) is not in the config, it's probably not the correct one
# Hence: copy
if ! cmp /etc/chrony/chrony.conf ~/.tue/installer/targets/amigo1/chrony.conf --quiet
then
    tue-install-info "Chrony config is probably not correct, will copy"
    
    # Backup old config
    sudo mv /etc/chrony/chrony.conf /etc/chrony/chrony.conf.backup
    
    # Copy new config
    sudo cp ~/.tue/installer/targets/amigo1/chrony.conf /etc/chrony/chrony.conf
    
    # Restart chrony
    sudo service chrony restart
fi

# UDEV rules
if [ ! -f /etc/udev/rules.d/0-hokuyo.rules ]
then
    tue-install-cp udev-rules/* /etc/udev/rules.d/
fi
