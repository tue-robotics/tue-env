tue-install-system-now mercurial

model_path=$HOME/data/gazebo_models
if [ ! -d $model_path ]
then
    hg clone https://bitbucket.org/osrf/gazebo_models $model_path
else
    cd $model_path
    hg pull -f
fi
