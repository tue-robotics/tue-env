# Setting up SSH support on Travis CI

The travis configuration needs changes to recurse submodules from private gitlab repository.

Steps to enable ssh support with travis and gitlab are as follows:

## On the local PC

1. Generate an ssh keypair to be used with CI

   ```bash
   mkdir -p ~/travis
   ssh-keygen -t rsa -b 4096 -N "" -f ~/travis/deploy_key
   ```

2. Add the ssh key to travis

   ```bash
   gem install travis
   cd <REPOSITORY>
   travis encrypt-file ~/travis/deploy_key --pro --add
   ```

   The last command will automatically generate and set two environment variables `encrypted_<STRING>_key` and
   `encrypted_<STRING>_iv`in the repository settings of travis. They are not to be removed.

3. Change the value of `-out` option in `before_install: - openssl` to `./deploy_key`

4. Commit the updated .travis.yml file and the generated file `deploy_key.enc`

### For targets repository

1. Append the following lines to .travis.yml under `before_install`

   ```bash
   - eval "$(ssh-agent -s)"
   - chmod 600 ./deploy_key
   - ssh-add ./deploy_key
   ```

### For package repository

1. Add the following lines to .travis.yml under `install`

   <!-- markdownlint-disable MD013 -->
   ```bash
   - SSH_KEY_PRIVATE=$(cat deploy_key)
   - bash install-package.sh --package=$PACKAGE --branch=$TRAVIS_BRANCH --commit=$TRAVIS_COMMIT --pullrequest=$TRAVIS_PULL_REQUEST --ssh --ssh-key="$SSH_KEY_PRIVATE"
   ```
   <!-- markdownlint-enable MD013 -->

## On GITLAB

1. In the repository Settings > CI/CD > Deploy Keys, add the public key from the file ~/travis/deploy_key.pub
