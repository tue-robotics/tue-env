pushd .

echo "Installing Torch"

git clone https://github.com/torch/distro.git ~/torch --recursive
cd ~/torch
bash install-deps
./install.sh
source ~/.bashrc

luarocks install dpnn
luarocks install nn
luarocks install optim
luarocks install csvigo
# luarocks install cunn #(only with CUDA)
luarocks install fblualib #(only for training a DNN)
luarocks install torchx #(only for training a DNN)

popd 