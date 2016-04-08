# Add repo because ffmpeg is not listed in the default trusty repo
repo=mystic-mirage/pycharm
grep -h "^deb.*$repo" /etc/apt/sources.list.d/* > /dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "Adding ppa:$repo"
    sudo add-apt-repository -y ppa:$repo
    sudo apt-get update
fi

tue-install-system pycharm-community
