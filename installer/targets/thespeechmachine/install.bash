# Install vbox if not installed
if ! dpkg -s virtualbox-4.3 &> /dev/null
then 
    URL='http://download.virtualbox.org/virtualbox/4.3.20/virtualbox-4.3_4.3.20-96996~Ubuntu~precise_amd64.deb'; FILE=`mktemp`; wget "$URL" -qO $FILE && sudo dpkg -i $FILE; rm $FILE
fi

# Download and unzip if vbox not there
mkdir -p ~/.vbox_images
if [ ! -d ~/.vbox_images/thespeechmachine ]
then
    wget --no-check-certificate http://roboticssrv.wtb.tue.nl/vbox_images/thespeechmachine.tar.gz -O- | tar -xzv --directory ~/.vbox_images
    vboxmanage registervm ~/.vbox_images/thespeechmachine/thespeechmachine.vbox
fi
