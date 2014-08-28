#! /bin/bash

if [ -z $1 ]
then
    echo "Usage: tue-create ros-pkg PACKAGE_NAME"
    return 1
fi

if [ -d $1 ]
then
    echo "Package '$1' already exists"
    return 1
fi

mkdir $1
cp ~/.tue/create/ros-pkg/package.xml $1
cp ~/.tue/create/ros-pkg/CMakeLists.txt $1
