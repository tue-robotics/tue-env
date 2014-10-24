#!/bin/bash

# .vimrc
if [ ! -f ~/.vimrc ]
then
	echo "installing a basic vimrc..."
    # install a TU/e .vimrc
	cp ~/.tue/installer/targets/vim/vimrc ~/.vimrc
fi

# install pathogen for automatic vim plugin loading
if [ ! -d ~/.vim/autoload ]
then
    # install pathogen.vim
    # https://github.com/tpope/vim-pathogen
    mkdir -p ~/.vim/autoload ~/.vim/bundle
    curl -LSso ~/.vim/autoload/pathogen.vim https://tpo.pe/pathogen.vim

    # Now you can install any plugin into a .vim/bundle/plugin-name/ folder
fi

# editorconfig
if [ ! -d ~/.vim/bundle/editorconfig-vim ]
then
    echo "installing editorconfig..."
    git clone https://github.com/editorconfig/editorconfig-vim.git ~/.vim/bundle/editorconfig-vim
fi

# set the default editor to vim
vimdir="/usr/bin/vim.basic"
if [ -f $vimdir ] # only if vim is installed
then
    current=`update-alternatives --query editor | grep Value | awk '{print $2}'`
    if [ "$current" = "/bin/nano" ]
    then
        echo -e "\nYou are using nano, let's setup vim\n"
        sudo update-alternatives --set editor $vimdir
    fi
fi

tue-install-system vim

# TODO: when should we run this (vim needs to be installed)
# sudo update-alternatives --set editor /usr/bin/vim.basic
