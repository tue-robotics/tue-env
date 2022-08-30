******************
CI Setup on GitLab
******************

Table of Contents
=================
#. `Setting up GitLab runners <runners_>`_

   #. `Docker executor <runners_docker_executor_>`_
   #. `Virtualbox executor <runners_virtualbox_executor_>`_
#. `Setting up CI/CD for a repository <repo_>`_

   #. `Deploy key setup <repo_deploy_key_setup_>`_
   #. `Adding CI config file to the repository <repo_ci_config_>`_

      #. `Multi-package repositories <repo_ci_config_multi-package_>`_
      #. `Upstream Mirroring <repo_ci_config_upstream_mirroring_>`_
      #. `Docker image and/or debian package creation <repo_ci_config_docker_debian_creation_>`_

.. _runners:

Setting up GitLab runners
=========================

Setting up GitLab runners on a Linux machine starts with installing the ``gitlab-runner`` package following the steps in the `documentation <https://docs.gitlab.com/runner/install/linux-repository.html>`_

.. _runners_docker_executor:

Docker Executor
---------------
#. Install docker engine on the machine
#. Register gitlab runner with docker-in-docker configuration, TLS support and Overlay2 driver enabled. `Refer <https://docs.gitlab.com/ee/ci/docker/using_docker_build.html#use-docker-in-docker-workflow-with-docker-executor>`_

   .. code-block:: bash

       sudo gitlab-runner register \
           --non-interactive \
           --url "<GITLAB_INSTANCE_URL>" \
           --registration-token "<PROJECT_REGISTRATION_TOKEN>" \
           --executor "docker" \
           --docker-image <DEFAULT_DOCKER_IMAGE> \
           --docker-privileged \
           --docker-volumes "/certs/client" \
           --description "<RUNNER_DESCRIPTION>" \
           --tag-list "<RUNNER_TAGS>" \
           --run-untagged="true" \
           --locked="false" \
           --access-level="not_protected"

   Add flag ``--docker-memory "<number><l>"`` where ``l`` is either of ``b``, ``k``, ``m`` or ``g``

.. _runners_virtualbox_executor:

Virtualbox Executor
-------------------
#. Install Virtualbox on the machine
#. Register a virtualbox runner

   .. code-block:: bash

       sudo gitlab-runner register \
           --non-interactive \
           --url "<GITLAB_INSTANCE_URL>" \
           --registration-token "<PROJECT_REGISTRATION_TOKEN>" \
           --executor "virtualbox" \
           --description "<RUNNER_DESCRIPTION>" \
           --tag-list "<RUNNER_TAGS>" \
           --run-untagged="true" \
           --locked="false" \
           --access-level="not_protected"

.. _repo:

Setting up CI/CD for a repository
=================================
Setting up GitLab CI/CD for a repository involves the following steps:

.. _repo_deploy_key_setup:

Deploy key setup
----------------
#. On the GitLab repository project page, hover over to **Settings** and click on **Repository**
    .. figure:: ./images/CI_GitLab_Setup-settings-repository.png
        :width: 750px
        :align: center
        :figclass: align-center
#. Scroll down to the **Deploy keys** section and click on **Expand**
    .. figure:: ./images/CI_GitLab_Setup-deploy-key-menu.png
        :width: 750px
        :align: center
        :figclass: align-center
#. Select the correct deploy key from **Privately accessible deploy keys** and click **Enable**
    .. figure:: ./images/CI_GitLab_Setup-deploy-key-selection.png
        :width: 750px
        :align: center
        :figclass: align-center
#. Check if the selected key was enabled in **Enabled deploy keys**
    .. figure:: ./images/CI_GitLab_Setup-enabled-deploy-keys.png
        :width: 750px
        :align: center
        :figclass: align-center

If using Mirror Upstream job
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For using the *Mirror Upstream* job, the enabled deploy key must have write permissions enabled:

#. Go to the **Enabled deploy keys** menu from the previous section and click on the **Edit** icon of the key
    .. figure:: ./images/CI_GitLab_Setup-edit-deployed-key.png
        :width: 750px
        :align: center
        :figclass: align-center
#. Check **Grant write permissions to this key** and click **Save changes**
    .. figure:: ./images/CI_GitLab_Setup-deploy-key-write-permissions.png
        :width: 750px
        :align: center
        :figclass: align-center

.. _repo_ci_config:

Adding CI config file to the repository
---------------------------------------

To enable GitLab CI/CD on a repository, it is necessary to add a ``.gitlab-ci.yml`` file to the root of the repository.
This file must at least contain the following content:

.. code-block:: yaml

    include:
      - project: 'avular/common-tools/package-manager/tue-env'
        file: '/ci/template.gitlab-ci.yml'

    Install, Build & Test:
        extends: .install_build_test
        variables:
            ROS_DISTRO: <DESIRED_ROS_DISTRO>

where ``<DESIRED_ROS_DISTRO>`` should be replaced with one of the supported ``ROS`` distro from the table below:

+------------------------+-----------------------------------+
| Name of ``ROS`` distro | Value of ``<DESIRED_ROS_DISTRO>`` |
+========================+===================================+
| Noetic Ninjemys        | ``noetic``                        |
+------------------------+-----------------------------------+
| Galactic Geochelone    | ``galactic``                      |
+------------------------+-----------------------------------+


.. _repo_ci_config_multi-package:

Multi-package repositories
^^^^^^^^^^^^^^^^^^^^^^^^^^
By default the CI job tries to build the package with the same name of the repository. When dealing with multi-package repositories, this may not be desired default behaviour.
To override the default behaviour add the variable ``PACKAGE_NAME`` with the desired package to the CI job.
The amount of jobs is not limited to one. For every package a new job can be created by extending ``.install_build_test`` with the ``PACKAGE_NAME`` variable set to the appropriate value

.. code-block:: yaml

    include:
      - project: 'avular/common-tools/package-manager/tue-env'
        file: '/ci/template.gitlab-ci.yml'

    Install, Build & Test:
        extends: .install_build_test
        variables:
            ROS_DISTRO: <DESIRED_ROS_DISTRO>
            PACKAGE_NAME: <DESIRED_ROS_PACKAGE>

.. _repo_ci_config_upstream_mirroring:

Upstream Mirroring
^^^^^^^^^^^^^^^^^^
To mirror an upstream repository the following content must also be in ``.gitlab-ci.yml``:

.. code-block:: yaml

    Mirror Upstream:
        extends: .mirror_upstream

.. _repo_ci_config_docker_debian_creation:

Docker image and/or debian package creation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To build docker images for a package add either or all of the following content to ``.gitlab-ci.yml``

.. code-block:: yaml

    Package Release [amd64]:
        extends: .package-release [amd64]
        variables:
            ROS_DISTRO: <DESIRED_ROS_DISTRO>
            PACKAGE: <DESIRED_PACKAGE>

    Package Release [arm64]:
        extends: .package-release [arm64]
        variables:
            ROS_DISTRO: <DESIRED_ROS_DISTRO>
            PACKAGE: <DESIRED_PACKAGE>

    Package Release:
        extends: .package-release
        needs: ['Package Release [amd64]', 'Package Release [arm64]']

The ``needs`` field should be adapated accordingly if only one of ``Package Release [amd64]`` and ``Package Release [arm64]``
are added to ``.gitlab-ci.yml``
