#! /bin/bash

if [ -z "$1" ]
then
    echo "Usage: tue-create ros-pkg PACKAGE_NAME [ DEPENDENCY1 DEPENDENCY2 ... ]"
    return 1
fi

if [ -d $1 ]
then
    echo "Package '$1' already exists"
    return 1
fi

deps=${@:2}

mkdir $1

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
# package.xml

package_file=$1/package.xml

echo '<?xml version="1.0"?>
<package>
  <name>ed_examples</name>
  <version>0.0.0</version>
  <description>No description available</description>

  <maintainer email="todo@tue.nl">todo</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
' > $package_file

for dep in $deps
do
    echo "  <build_depend>$dep</build_depend>" >> $package_file
    echo "  <run_depend>$dep</run_depend>" >> $package_file
    echo "" >> $package_file
done

echo '</package>' >> $package_file


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
# CMakeLists.txt

cmake_file=$1/CMakeLists.txt

echo "cmake_minimum_required(VERSION 2.8.3)
project($1)
" > $cmake_file

echo 'find_package(catkin REQUIRED COMPONENTS' >> $cmake_file

for dep in $deps
do
    echo "    $dep" >> $cmake_file
done
echo ')

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

add_libary(library_name
    src/lib_source_file1.cpp
    ...
)

add_executable(exec_name
    src/source_file1.cpp
    ...
)

' >> $cmake_file



