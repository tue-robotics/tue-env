if [ ! -f /etc/init.d/autoroscore ] || [[ `diff ~/.tue/installer/targets/amigo1/autoroscore /etc/init.d/autoroscore` != "" ]]
then
    sudo cp ~/.tue/installer/targets/amigo1/autoroscore /etc/init.d/autoroscore
fi

if [ ! -f /etc/init.d/autoroslaunch ] || [[ `diff ~/.tue/installer/targets/amigo1/autoroslaunch /etc/init.d/autoroslaunch` != "" ]]
then
    sudo cp ~/.tue/installer/targets/amigo1/autoroslaunch /etc/init.d/autoroslaunch
fi

