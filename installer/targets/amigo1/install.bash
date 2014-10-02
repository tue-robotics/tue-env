# --------------------------------------------------------------------------------
# - Set automatic start-up of roscore and astart on boot

if [[ `diff ~/.tue/installer/targets/amigo1/autoroscore /etc/init.d/autoroscore` != "" ]]
then
    sudo cp ~/.tue/installer/targets/amigo1/autoroscore /etc/init.d/autoroscore
fi

if [[ `diff ~/.tue/installer/targets/amigo1/autoroslaunch /etc/init.d/autoroslaunch` != "" ]]
then
    sudo cp ~/.tue/installer/targets/amigo1/autoroslaunch /etc/init.d/autoroslaunch
fi
