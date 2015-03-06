#! /bin/bash

mkdir -p $TUE_ENV_DIR/cmake

# Generate cmake-files
for pkg in `ls $TUE_ENV_DIR/pkgs`
do
	if [ -f $TUE_ENV_DIR/pkgs/$pkg/CMakeLists.txt ]
	then
        libs=`find $TUE_ENV_DIR/build/$pkg -name *.so -o -name *.a`
		cmake_file=$TUE_ENV_DIR/cmake/${pkg}Config.cmake
		echo "set(${pkg}_INCLUDE_DIRS $TUE_ENV_DIR/pkgs/$pkg/include)" > $cmake_file
		echo "set(${pkg}_LIBRARIES $libs)" >> $cmake_file
	fi
done

