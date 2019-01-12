url="https://github.com/yu4u/age-gender-estimation/releases/download/v0.5/weights.28-3.73.hdf5"
old_dest=~/src/keras_models/age_gender
dest=~/data/keras_models/age_gender

# move old dir to new destination
if [ -d $old_dest ]
then
    mkdir -p $(dirname $dest)
    mv $old_dest $dest
    if [ -z "$(ls -A $(dirname $old_dest))" ]
    then
        rm -r $(dirname $old_dest)
    fi
fi

if [ ! -d $dest ]
then
    tue-install-debug "wget $url -P $dest"
    wget $url -P $dest
fi
