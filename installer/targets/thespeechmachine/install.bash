# Install vbox if not installed
if ! dpkg -s virtualbox &> /dev/null
then 
    sudo apt-get install virtualbox
fi

# Download and unzip if vbox not there
mkdir -p ~/.vbox_images
if [ ! -d ~/.vbox_images/thespeechmachine ]
then
    wget --no-check-certificate http://roboticssrv.wtb.tue.nl/vbox_images/thespeechmachine.tar.gz -O- | tar -xzv --directory ~/.vbox_images
    vboxmanage registervm ~/.vbox_images/thespeechmachine/thespeechmachine.vbox
fi
