# tue-env
[![pipeline status](https://gitlab.com/avular/common-tools/package-manager/tue-env/badges/master/pipeline.svg)](https://gitlab.com/avular/common-tools/package-manager/tue-env/-/commits/master)

Package manager that can be used to install (ROS) dependencies

## Installation

#### Ubuntu 18.04/20.04
Standard tue-env installation with targets from [tue-env-targets](https://gitlab.com/avular/common-tools/package-manager/tue-env-targets)

##### Add SSH key to gitlab to gain access to this repository
Add the public part of your ssh-key (`cat ~/.ssh/<KEY_NAME>.pub`, where `<KEY_NAME>` is the name of your ssh-key) to Gitlab > Settings > SSH Key and add key

To generate a new ssh keypair:
```bash
sudo apt-get install ssh
ssh-keygen
cat ~/.ssh/<KEY_NAME>.pub
```

##### Installing the tue-env
Download the bootstrap.bash file manually from gitlab.com (https://gitlab.com/avular/common-tools/package-manager/tue-env/-/blob/master/installer/bootstrap.bash)

1. Bootstrap the package manager 
   ```bash
   source [DOWNLOAD DIRECTORY]/bootstrap.bash
   ```
2. Install target(s)
   ```bash
   tue-get install idrive-robot # for installation on the i-drive
   # Or
   tue-get install idrive-dev   # includes idrive-robot plus extra simulation tools
   ```
3. Build sources
   ```bash
   tue-make
   source ~/.bashrc  # Or open a new terminal
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

### Pre-built docker image

1. Pull the latest tue-env docker image from GitLab
```bash
echo <GITLAB_PASSWORD/TOKEN> | docker login registry.gitlab.com -u <GITLAB_USERNAME> --password-stdin
docker pull registry.gitlab.com/avular/common-tools/package-manager/tue-env:latest
```

2. Start an interactive docker container that autoremoves upon exit, with local ssh keys mounted
```bash
docker run --rm -it --network host --mount type=bind,source=$HOME/.ssh,target=/tmp/.ssh registry.gitlab.com/avular/common-tools/package-manager/tue-env:latest
```

3. Enable ssh agent inside the container and add the mounted ssh key(s) to it
```bash
eval $(ssh-agent -s)
ssh-add /tmp/.ssh/<PRIVATE_SSH_KEYNAME>
source ~/.bashrc
```

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
    type: [git/hg/svn]
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
  indigo:
    source:
      type: git
      url: <Repository URL>
  default:
    source: null
```
Both ROS distro specific as default can be 'null'. Prevered usage is default for current and feature distributions and exceptions for old distributions.

#### Catkin package install
```yaml
- type: catkin
  source:
    type: [git/hg/svn]
    url: <Repository URL>
    sub-dir: <Sub directory of the repository> (Optional field)
    version: <Version to be installed> (Optional field)
```
This target type is similar to a ROS source target with only difference being a catkin package is independent of any ROS
dependencies and solely depends on catkin. Examples of such packages are `librealsense`, `gtsam`, etc.

#### Target / System / PIP / PIP2 / PIP3 / PPA / Snap / DPKG / Empty
```yaml
- type: [target/system/pip/pip2/pip3/ppa/snap/dpkg/empty]
  name: <Name of the candidate>
```

Depending on Ubuntu distribution:

```yaml
- type: [target/system/pip/pip2/pip3/ppa/snap/dpkg/empty]
  xenial:
    name: [null/<Name of the candidate>]
  default:
    name: [null/<Name of the candidate>]
```

Both Ubuntu distribution specific as default can be 'null'. Prevered usage is default for current and feature distributions and exceptions for old distributions.

#### (Target / System / PIP / PIP2 / PIP3 / PPA / Snap)-now

The default installation method for targets of type `system`, `pip(2/3)`, `ppa` and `snap` is to collect all such targets in a list and install them simultaneously at the end of the `tue-get install` procedure. To install such a dependency immediately for a specific target, use the target type as `X-now`:

```yaml
- type: [target/system/pip/pip2/pip3/ppa/snap]-now
  name: <Name of the candidate>

- type: [target/system/pip/pip2/pip3/ppa/snap/dpkg]
  name: <Name of the candidate>
```

`target-now` will install a target directly recursively. So also all its dependencies will be installed directly, by converting them from `XX` to `XX-now`. Except `ROS` and `DPKG` are excluded. ROS dependencies are excluded, because ROS packages should only be used at runtime, because it requires either a compilation and/or resourcing the workspace.
It is preferred to include these `-now` dependencies in `install.yaml`. Only use the corresponding bash function in `install.bash` if no other solution is possible.

#### GIT / HG / SVN

```yaml
- type: [git/hg/svn]
  url: <url>
  path: <path/where/to/clone>
  version: [branch/commit/tag] (Optional field)
```

### `tue-install` functions for `install.bash`

The following functions provided with `tue-env` must be preferred over any
generally used methods of installing packages:

| Function Name                   | Description                                                                                    |
|---------------------------------|------------------------------------------------------------------------------------------------|
| `tue-install-add-text`          | To add/replace text in a file with `sudo` taken into account                                   |
| `tue-install-apt-get-update`    | Make sure that during next `tue-install-system-now` call `apt-get` is updated                  |
| `tue-install-cp`                | Analogous to `cp` but takes `sudo` into account and the source should be relative to target    |
| `tue-install-dpkg`              | To install a debian dpkg file                                                                  |
| `tue-install-git`               | To install a git repository                                                                    |
| `tue-install-pip`               | To add a python pip2 package to a list to be installed at the end (deprecated)                 |
| `tue-install-pip2`              | To add a python pip2 package to a list to be installed at the end                              |
| `tue-install-pip3`              | To add a python pip3 package to a list to be installed at the end                              |
| `tue-install-pip-now`           | To install python pip2 package, but ignores it if already installed (deprecated)               |
| `tue-install-pip2-now`          | To install python pip2 package, but ignores it if already installed                            |
| `tue-install-pip3-now`          | To install python pip3 package, but ignores it if already installed                            |
| `tue-install-ppa`               | To add one PPA/DEB to a list to be added with `apt-add-repository` at the end, before apt-get  |
| `tue-install-ppa-now`           | To add a PPA/DEB with `apt-add-repository`, use ^ inside of a DEB and spaces between items     |
| `tue-install-snap`              | To add a snap package to a list to be installed at the end                                     |
| `tue-install-snap-now`          | To install a snap                                                                              |
| `tue-install-svn`               | To install a svn repository                                                                    |
| `tue-install-system`            | To add `deb` package to a list of packages to be installed at the end with `apt-get`           |
| `tue-install-system-now`        | To install `deb` packages with `apt-get` right away, but ignores it if already installed       |
| `tue-install-get-releases`      | To get a released asset from a github repository and place it in the requested directory       |

The input arguments for each of the above mentioned commands can be found by
simply executing the command in a bash session (provided tue-env is correctly
installed).

A general remark about the order of preference of package repositories:

system > ppa > pip2 = pip3 > snap > git > hg > svn > dpkg (> pip, deprecated)

### Adding SSH support to a repository
See [this](ci/README.md)
