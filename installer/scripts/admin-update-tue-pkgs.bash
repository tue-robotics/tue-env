# Displays the value of a given attribute returned by svn info
# Usage: svn-info <DIRECTORY> <ATTRIBUTE>
# Example: svn-info ~/path/to/my/svn URL
#   gives back the URL of local check-out at ~/path/to/my/svn
function svn-info {
    echo `svn info $1 | grep "^$2:" | sed "s/$2: //g"`
}

targets_dir=~/.tue/installer/targets

# Create tue-all target
mkdir -p $targets_dir/tue-all
echo '' > $targets_dir/tue-all/install

# Update ROSBUILD packages
a=`find ~/svn/tue/trunk-rosbuild -name manifest.xml`
for path in $a
do
    # Get rid of /manifest.xml
    pkg_path=${path%/manifest.xml}

    # Remove everything before the last slash
    pkg_name=${pkg_path##*/};

    echo "mkdir -p $targets_dir/${pkg_name}-ros"

    url=`svn-info $pkg_path URL`

    mkdir -p $targets_dir/${pkg_name}-ros

    echo $url

    echo "tue-install-ros-rosbuild svn $url" > $targets_dir/${pkg_name}-ros/install

    # Add target to tue-all
    echo "tue-install-target ${pkg_name}-ros" >> $targets_dir/tue-all/install
done

# Update CATKIN packages
a=`find ~/svn/tue/trunk-catkin -name package.xml`
for path in $a
do
    # Get rid of /package.xml
    pkg_path=${path%/package.xml}

    # Remove everything before the last slash
    pkg_name=${pkg_path##*/};

    echo "mkdir -p $targets_dir/${pkg_name}-ros"

    url=`svn-info $pkg_path URL`

    mkdir -p $targets_dir/${pkg_name}-ros

    echo $url

    echo "tue-install-ros svn $url" > $targets_dir/${pkg_name}-ros/install

    # Add target to tue-all
    echo "tue-install-target ${pkg_name}-ros" >> $targets_dir/tue-all/install
done

