
function tue-check-version
{
	if [ -z "$1" ]
    then
        echo "Invalid tue-check-version: needs package as argument."
        return 1
    fi
	
	local ros_pkg_name=$1
	
	local ros_pkg_dir
    ros_pkg_dir="$ROS_PACKAGE_INSTALL_DIR"/"$ros_pkg_name"
	
	local pkg_xml="$ros_pkg_dir"/package.xml
	
	local deps
    deps=$("$TUE_INSTALL_SCRIPTS_DIR"/parse_package_xml_2.py "$pkg_xml")
    for d in $deps
	do
		echo "depends on $d"
	done
}

function _tue-dir-version
{
    [ -d "$1" ] || return 1

    local fs
    fs=$(ls "$1")
    for f in $fs
    do
        local pkg
        pkg=$f
        local version
        version=$(rosversion "$f")
        echo "package $pkg has version $version" 
    done
}

# ----------------------------------------------------------------------------------------------------

function tue-version
{
    _tue-dir-version "$TUE_SYSTEM_DIR"/src
}

# Initialize
TUE_INSTALL_SCRIPTS_DIR=$TUE_DIR/installer

# Initialize
ROS_PACKAGE_INSTALL_DIR=$TUE_SYSTEM_DIR/src
