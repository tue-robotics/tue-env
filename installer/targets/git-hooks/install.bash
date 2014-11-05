# install the global hooks
if [ ! -d ~/.git_hooks ]
then
    echo "linking ~/.git_hooks"
    ln -s ~/.tue/installer/targets/git-hooks/git_hooks ~/.git_hooks
fi
