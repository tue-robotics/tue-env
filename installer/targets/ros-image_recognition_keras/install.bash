url="https://github.com/yu4u/age-gender-estimation/releases/download/v0.5/weights.28-3.73.hdf5"
dest=~/src/keras_models/age_gender
if [ ! -d $dest ]
then
    wget $url -P $dest
fi
