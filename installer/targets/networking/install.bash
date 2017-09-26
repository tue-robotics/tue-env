## nsswitch
set -e

# Check config file
if ! cmp /etc/nsswitch.conf ~/.tue/installer/targets/networking/nsswitch.conf --quiet
then
    echo "nsswitch config is probably not correct, will copy"
    
    # Copy and backup old config
    sudo install --backup=numbered --compare --verbose ~/.tue/installer/targets/networking/nsswitch.conf /etc/nsswitch.conf
fi
