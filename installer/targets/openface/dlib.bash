pushd .

echo "Installing DLib"

mkdir -p ~/src
cd ~/src
wget https://github.com/davisking/dlib/releases/download/v18.16/dlib-18.16.tar.bz2
tar xf dlib-18.16.tar.bz2
cd dlib-18.16/python_examples
mkdir build
cd build
cmake ../../tools/python
cmake --build . --config Release
sudo cp dlib.so /usr/local/lib/python2.7/dist-packages

export LD_LIBRARY_PATH=/usr/lib/openblas-base/


#From http://serverfault.com/questions/201709/how-to-set-ld-library-path-in-ubuntu
#To define this variable, simply use (on the shell prompt):
export LD_LIBRARY_PATH="/usr/lib/openblas-base/"

#To make it permanent, you can edit the ldconfig files. First, create a new file such as:
sudo touch /etc/ld.so.conf.d/libopenblas-base.conf

#Second, add the path in the created file
sudo sh -c 'echo "/usr/lib/openblas-base/" > /etc/ld.so.conf.d/libopenblas-base.conf'

#Finally, run ldconfig to update the cache.
sudo ldconfig

echo "dlib.bash finished"

popd