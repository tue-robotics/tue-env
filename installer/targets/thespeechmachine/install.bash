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

red='\033[0;31m'
NC='\033[0m' # No Color
echo -e "${red}\nDo the following things to make the vbox work:\n\n- Log into this pc with ssh -X user@pc-name\n- virtualbox\n- Make sure you select pulseaudio and the correct driver\n- Set the correct ip and bridge it!\n- Get the latest version from github (cmd; cd dragonfly_speech_recognition; git pull)\n- Reboot pc\n- Check if it works\n- Store a snapshop\n\nEverybody is happy!\n\n${NC}"


