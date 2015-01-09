# tue-env
Package manager that can be used to install (ROS) dependencies

## Installation
Run the following commands in a shell:
```bash
export TUE_ENV=hydro && source <(wget -O- https://raw.githubusercontent.com/tue-robotics/tue-env/master/installer/scripts/bootstrap)
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
