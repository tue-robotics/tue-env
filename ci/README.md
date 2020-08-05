# Setting up SSH support on Travis CI
The travis configuration needs changes to recurse submodules from private gitlab repository. 

Steps to enable ssh support with travis and gitlab are as follows:

## On the local PC
1. Generate a ssh keypair to be used with CI
   ```
   mkdir -p ~/travis
   ssh-keygen -t rsa -b 4096 -N "" -f ~/travis/deploy_key
   ```

1. Add the ssh key to travis
   ```
   gem install travis
   cd <REPOSITORY>
   travis encrypt-file ~/travis/deploy_key --pro --add
   ```
   The last command will automatically generate and set two environment variables `encrypted_<STRING>_key` and
   `encrypted_<STRING>_iv`in the repository settings of travis. They are not to be removed.

1. Change the value of `-out` option in `before_install: - openssl` to `./deploy_key`

1. Commit the updated .travis.yml file and the generated file `deploy_key.enc`

### For targets repository
1. Append the following lines to .travis.yml under `before_install`
   ```
   - eval "$(ssh-agent -s)"
   - chmod 600 ./deploy_key
   - ssh-add ./deploy_key
   ```

### For package repository
1. Add the following lines to .travis.yml under `install`
   ```
   - SSH_KEY_PRIVATE=$(cat deploy_key)
   - bash install-package.sh --package=$PACKAGE --branch=$TRAVIS_BRANCH --commit=$TRAVIS_COMMIT --pullrequest=$TRAVIS_PULL_REQUEST --ssh --ssh-key="$SSH_KEY_PRIVATE"
   ```

## On GITLAB
1. In the repository Settings > CI/CD > Deploy Keys, add the public key from the file ~/travis/deploy_key.pub
