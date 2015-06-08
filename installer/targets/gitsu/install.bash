# gitsu is installed as a ruby gem
hash gem 2> /dev/null || sudo apt-get install ruby
hash git-su 2> /dev/null || sudo gem install gitsu

# install the authors file
if [ ! -f ~/.gitsu ]
then
    echo "linking ~/.gitsu"
    ln -s ~/.tue/installer/targets/gitsu/gitsu.txt ~/.gitsu
fi
