if [ ! -d ~/GPSRCmdGen ]; then
	pushd .

	echo "Installing GPSRCmdGen"

	echo "Required for GPSRCmdGen: mono-complete"
	sudo apt-get install mono-complete

	echo "Cloning GPSRCmdGen from git repository"
	git clone http://github.com/kyordhel/GPSRCmdGen.git ~/GPSRCmdGen --recursive

	echo "Making GPSRCmdGen"
	cd ~/GPSRCmdGen
	make

	popd
fi
