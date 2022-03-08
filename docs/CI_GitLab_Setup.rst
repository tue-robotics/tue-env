******************
CI Setup on GitLab
******************
Setting up GitLab CI/CD for a repository involves the following steps:

Deploy key setup
================
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
----------------------------
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

Adding CI config file to the repository
=======================================
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

To mirror an upstream repository the following content must also be in ``.gitlab-ci.yml``:

.. code-block:: yaml

    Mirror Upstream:
        extends: .mirror_upstream
