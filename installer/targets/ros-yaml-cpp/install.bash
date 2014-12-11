ubuntu_version=$(lsb_release -sr)

if [ $ubuntu_version == 12.04 ]
then
    tue-install-system yaml-cpp
else
    tue-install-system libyaml-cpp-dev
fi
