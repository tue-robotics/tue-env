# tue-env
Package manager that can be used to install (ROS) dependencies

## Installation

#### Ubuntu 16.04
```bash
source <(wget -O- https://raw.githubusercontent.com/tue-robotics/tue-env/master/installer/scripts/bootstrap-ros-kinetic)
tue-get install tue-dev #or
tue-get install tue-dev-full #tue-dev plus extra tools
source ~/.bashrc
```

## Usage

With `tue-get` you can install various targets which mostly are ros packages. 
The list of packages can be seen [here](installer/targets).

```bash
tue-get install <TARGET_NAME>
```
For example, so install a default developement installation for working with 
TU/e robots, run the following command:
```bash
tue-get install tue-dev
```
So automatically source the installer and corresponding workspaces run the following
command in a shell.
```bash
echo -e "export TUE_ENV=$TUE_ENV\nsource ~/.tue/setup.bash" >> ~/.bashrc
```
