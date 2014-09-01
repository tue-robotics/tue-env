source ~/.tue/setup/tue.bash_functions
if [ -f ~/.tue/setup/target_setup.bash ]
then
    source ~/.tue/setup/target_setup.bash
else
    tue-setup
fi
