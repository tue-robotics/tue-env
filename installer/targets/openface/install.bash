pushd . 
mkdir ~/openface
cd ~/openface
git clone https://github.com/cmusatyalab/openface.git --recursive
sudo pip install numpy pandas scipy scikit-learn scikit-image

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
bash $DIR/dlib.bash

bash $DIR/torch.bash

cd ~/openface
sudo python2 setup.py install

models/get-models.sh 

popd

if [[ $(python -c "import dlib") -eq 1 ]]; then
	echo "DLIB is not properly installed"
fi

if [[ $(python -c "import cv2") -eq 1 ]]; then
	echo "opencv2 is not properly installed"
fi

if [[ $(python -c "import openface") -eq 1 ]]; then
	echo "openface is not properly installed"
fi