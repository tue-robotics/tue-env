#! /bin/bash

if [ -z "$1" ]
then
    echo "Usage: tue-create ros-pkg PACKAGE_NAME [ DEPENDENCY1 DEPENDENCY2 ... ]"
    return 1
fi

pkg_name=$1
pkg_dir=$TUE_ENV_DIR/sources/local/$pkg_name

if [ -d $pkg_dir ]
then
    echo "Package '$pkg_name' already exists"
    return 1
fi

if [ -f ./$pkg_name ]
then
    echo "File '$pkg_name' already exists"
    return 1
fi

if [ -d ./$pkg_name ]
then
    echo "Directory '$pkg_name' already exists"
    return 1
fi

deps=${@:2}

mkdir -p $pkg_dir
ln -s $pkg_dir ./$pkg_name

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
# package.xml

package_file=$pkg_dir/package.xml

echo '<?xml version="1.0"?>
<package>' > $package_file
echo "  <name>$pkg_name</name>" >> $package_file
echo '  <version>0.0.0</version>
  <description>No description available</description>

  <maintainer email="todo@tue.nl">todo</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
' >> $package_file

for dep in $deps
do
    echo "  <build_depend>$dep</build_depend>" >> $package_file
    echo "  <run_depend>$dep</run_depend>" >> $package_file
    echo "" >> $package_file
done

echo '</package>' >> $package_file


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
# CMakeLists.txt

cmake_file=$pkg_dir/CMakeLists.txt

echo "cmake_minimum_required(VERSION 2.8.3)
project($pkg_name)
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
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# add_library(library_name
#     src/lib_source_file1.cpp
#     ...
# )
# target_link_libraries(library_name ${catkin_LIBRARIES})

# add_executable(exec_name
#     src/source_file1.cpp
#     ...
# )
# target_link_libraries(exec_name ${catkin_LIBRARIES})

' >> $cmake_file



