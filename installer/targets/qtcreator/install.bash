DIRECTORY='/opt/Qt'
if [ ! -d "$DIRECTORY" ]; then
    wget http://download.qt.io/official_releases/online_installers/qt-unified-linux-x64-online.run -O /tmp/qt-open-source-installer
    chmod +x /tmp/qt-open-source-installer

    echo ""
    echo "Choose $DIRECTORY as install directory!"
    echo ""

    sudo /tmp/qt-open-source-installer
fi

