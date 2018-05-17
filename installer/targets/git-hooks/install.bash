# install the global hooks
git config --global core.hooksPath ~/.tue/installer/targets/git-hooks/git_hooks/

if dpkg --compare-versions $(git --version | awk '{print $3}') lt 2.9; then
    tue-install-warning "Please perform a
    sudo apt install git
afterwards to upgrade the git version to allow git-hooks"
fi
