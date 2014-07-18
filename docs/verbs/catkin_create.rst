``catkin create`` -- Create Packages
====================================

This verb enables you to quickly create workspace elements like boilerplate Catkin packages.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

  usage: catkin create [-h] {pkg} ...

  Creates a catkin workspace

  positional arguments:
    {pkg}       sub-command help
      pkg       Create a catkin package.

  optional arguments:
    -h, --help  show this help message and exit

.. code-block:: text

    usage: catkin create pkg [-h] [--rosdistro ROSDISTRO] [-v MAJOR.MINOR.PATCH]
                             [-l LICENSE] [-m NAME EMAIL] [-a NAME EMAIL]
                             [-d DESCRIPTION] [--catkin-deps [DEP [DEP ...]]]
                             [--system-deps [DEP [DEP ...]]]
                             [--boost-components [COMP [COMP ...]]]
                             PKGNAME

    Create a new Catkin package. Note that while the default options used by this
    command are sufficient for prototyping and local usage, it is important that
    any publically-available packages have a valid license and a valid maintainer
    e-mail address.

    positional arguments:
      PKGNAME               The name of the package to create. This name should be
                            completely lower-case with individual words separated
                            by undercores.

    optional arguments:
      -h, --help            show this help message and exit
      --rosdistro ROSDISTRO
                            The ROS distro (default: environment variable
                            ROS_DISTRO if defined)

    Package Metadata:
      -v MAJOR.MINOR.PATCH, --version MAJOR.MINOR.PATCH
                            Initial package version. (default 0.0.0)
      -l LICENSE, --license LICENSE
                            The software license under which the code is
                            distributed, such as BSD, MIT, GPLv3, or others.
                            (default: "TODO")
      -m NAME EMAIL, --maintainer NAME EMAIL
                            A maintainer who is responsible for the package.
                            (default: [username, username@todo.todo]) (multiple
                            allowed)
      -a NAME EMAIL, --author NAME EMAIL
                            An author who contributed to the package. (default: no
                            additional authors) (multiple allowed)
      -d DESCRIPTION, --description DESCRIPTION
                            Description of the package. (default: empty)

    Package Dependencies:
      --catkin-deps [DEP [DEP ...]], -c [DEP [DEP ...]]
                            The names of one or more Catkin dependencies. These
                            are Catkin-based packages which are either built as
                            source or installed by your system's package manager.
      --system-deps [DEP [DEP ...]], -s [DEP [DEP ...]]
                            The names of one or more system dependencies. These
                            are other packages installed by your operating
                            system's package manager.

    C++ Options:
      --boost-components [COMP [COMP ...]]
                            One or more boost components used by the package.
