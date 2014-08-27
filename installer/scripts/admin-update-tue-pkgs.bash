# Displays the value of a given attribute returned by svn info
# Usage: svn-info <DIRECTORY> <ATTRIBUTE>
# Example: svn-info ~/path/to/my/svn URL
#   gives back the URL of local check-out at ~/path/to/my/svn
function svn-info {
    echo `svn info $1 | grep "^$2:" | sed "s/$2: //g"`
}

targets_dir=~/.tue/installer/targets

# Update CATKIN packages
a=`find ~/svn/tue/trunk-catkin -name package.xml`
for path in $a
do
    # Get rid of /package.xml
    pkg_path=${path%/package.xml}

    # Remove everything before the last slash
    pkg_name=${pkg_path##*/};

    url=`svn-info $pkg_path URL`

    mkdir -p $targets_dir/ros-${pkg_name}

    echo $url

    echo """- type: ros
  source:
      type: svn
      url: $url
""" > $targets_dir/ros-${pkg_name}/install.yaml
done

