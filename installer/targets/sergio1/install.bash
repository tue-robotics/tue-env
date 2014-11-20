# --------------------------------------------------------------------------------
# - Set automatic start-up of roscore and astart on boot

if [[ `diff ~/.tue/installer/targets/sergio1/autoroscore /etc/init.d/autoroscore` != "" ]]
then
    sudo cp ~/.tue/installer/targets/sergio1/autoroscore /etc/init.d/autoroscore
fi

if [[ `diff ~/.tue/installer/targets/sergio1/autoroslaunch /etc/init.d/autoroslaunch` != "" ]]
then
    sudo cp ~/.tue/installer/targets/sergio1/autoroslaunch /etc/init.d/autoroslaunch
fi

# --------------------------------------------------------------------------------
# - Set rsettings file

tue-install-cp rsettings_file ~/.tue/.rsettings

