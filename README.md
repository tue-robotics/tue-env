# tue-env
Package manager that can be used to install (ROS) dependencies

## Installation

#### Ubuntu 16.04/18.04
Standard tue-env installation with targets from [tue-env-targets](https://github.com/tue-robotics/tue-env-targets)

```bash
source <(wget -O - https://raw.githubusercontent.com/tue-robotics/tue-env/master/installer/bootstrap.bash)
tue-get install tue-dev #or
tue-get install tue-dev-full #tue-dev plus extra tools
tue-make
source ~/.bashrc
```

#### Customization
A customized targets repository can be setup with this package manager (currently only one git repository is supported). If `tue-env` is already installed, to setup the targets repository run:
```bash
tue-env init-targets [ENVIRONMENT] <targets_repo_git_url>
```
else first setup `tue-env` by manually following the procedure in the bootstrap
script.

## Usage

With `tue-get` you can install various targets which mostly are ros packages.
The list of packages can be seen [here](https://github.com/tue-robotics/tue-env-targets).

```bash
tue-get install <TARGET_NAME>
```
For example, to install a default developement installation for working with
TU/e robots, run the following command:
```bash
tue-get install tue-dev
```

**Note:** Any ROS package which has a source installation must be built. In the
current implementation of `tue-get` this doesn't happen automatically. However
we provide an alias to `catkin build` as `tue-make` which would build the
`tue-env` workspace.

Upon executing the installation instructions mentioned in the previous section, `~/.tue/setup.bash` is automatically added in `.bashrc`. Sourcing `.bashrc` would make `tue-env` available to the bash session.

## Guidelines on creating a new target
A target can consist of the following three files:
1. `install.yaml`
2. `install.bash`
3. `setup`

Installation happens in the above order. First `install.yaml` is parsed and the
instructions in it are executed. Then `install.bash` is executed. This must have
commands/instructions that cannot be specified in the YAML file. Lastly, the
`setup` file is sourced in the bash environment by `setup.bash` of tue-env.

Any target dependencies that can be specified in `install.yaml` as other targets
or installable packages must be specified there. They should not be moved to
`install.bash`as`tue-env` has many controls in place to parse the YAML file.

### Naming conventions
Name of the target must start with `ros-` only if it is a `catkin` (ROS) package. It's `install.yaml` file must be in the format of [ROS target](#ros-package-install).

### Writing `install.yaml`
| Symbol | Convention                             |
|--------|----------------------------------------|
| []     | Alternate options                      |
| <>     | Input argument required with the field |

Some fields are mentioned to be optional.

Taking the above into account, the following combinations for `install.yaml` are possible:

#### ROS package install
1. From source
```yaml
- type: ros
  source:
    type: git [svn]
    url: <Repository URL>
    sub-dir: <Sub directory of the repository> (Optional field)
    version: <Version to be installed> (Optional field)
```
2. From system
```yaml
- type: ros
  source:
    type: system
    name: <Package name>
```
3. Depending on ROS distro
```yaml
- type: ros
  kinetic:
    source:
      type: system
      name: <Package name>
  default:
    source:
      type: git
      url: <Repository URL>
```

#### Target / System / PIP / PPA / Snap / DPKG / Empty
```yaml
- type: [target/system/pip/ppa/snap/dpkg/empty]
  name: <Name of the candidate>
```

### `tue-install` functions for `install.bash`
The following functions provided with `tue-env` must be preferred over any
generally used methods of installing packages:

| Function Name            | Description                                                                          |
|--------------------------|--------------------------------------------------------------------------------------|
| `tue-install-add-text`   | To add/replace text in a file with `sudo` taken into account                         |
| `tue-install-cp`         | Analogous to `cp` but takes `sudo` into account and the source should be relative to target |
| `tue-install-dpkg`       | To install a debian dpkg file                                                        |
| `tue-install-git`        | To install a git repository                                                          |
| `tue-install-pip`        | To install a python pip package                                                      |
| `tue-install-ppa`        | To add PPA with `apt-add-repository`                                                 |
| `tue-install-snap`       | To install a snap                                                                    |
| `tue-install-svn`        | To install a svn repository                                                          |
| `tue-install-system`     | To add `deb` package to a list of packages to be installed at the end with `apt-get` |
| `tue-install-system-now` | To install `deb` packages with `apt-get` right away                                  |

The input arguments for each of the above mentioned commands can be found by
simply executing the command in a bash session (provided tue-env is correctly
installed).

A general remark about the order of preference of package repositories:

system > ppa > git > svn > pip > snap > dpkg
