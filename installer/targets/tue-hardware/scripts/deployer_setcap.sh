#!/bin/bash
echo "======================"
echo "SETCAP SCRIPT - KOEN BUYS"
# Modified by Tim Clephas
echo "======================"
echo "DANGER: this script is created to reset the capabilities of the deployer-gnulinux in OCL "

deployer=$(rospack find ocl)/bin/deployer-gnulinux
cdeployer=$(rospack find ocl)/bin/cdeployer-gnulinux
corbadeployer=$(rospack find ocl)/bin/deployer-corba-gnulinux
luadeployer=$(rospack find ocl)/bin/rttlua-gnulinux

if [ ! -f $deployer ]
then
    echo "Deployer does not exist"
    exit
fi

tue-install-system-now libcap2-bin

sudo setcap cap_net_raw+ep $deployer cap_net_raw+ep $cdeployer cap_net_raw+ep $corbadeployer cap_net_raw+ep $luadeployer

echo "should be finished now : don't risk to do this on other files!"
