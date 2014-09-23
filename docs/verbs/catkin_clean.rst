``catkin clean`` -- Clean Build Products
========================================

The ``clean`` verb makes it easier and safer to clean various products of a catkin
workspace. In addition to removing entire **build**, **devel**, and **install spaces**,
it also gives you more fine-grained control over removing just parts of these
directories.

The ``clean`` verb is context-aware, but in order to work, it must be given the path
to an initialized catkin workspace, or called from a path contained in an initialized
catkin workspace. This is because the paths to the relevant spaces are contained in a
workspace's metadata directory.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    usage: catkin clean [-h] [--workspace WORKSPACE] [--profile PROFILE] [-a] [-b]
                        [-d] [-i] [-c] [-o]

    Deletes various products of the build verb.

    optional arguments:
      -h, --help            show this help message and exit
      --workspace WORKSPACE, -w WORKSPACE
                            The path to the catkin_tools workspace or a directory
                            contained within it (default: ".")
      --profile PROFILE     The name of a config profile to use (default: active
                            profile)

    Basic:
      Clean workspace subdirectories.

      -a, --all             Remove all of the *spaces associated with the given or
                            active profile. This will remove everything but the
                            source space and the hidden .catkin_tools directory.
      -b, --build           Remove the buildspace.
      -d, --devel           Remove the develspace.
      -i, --install         Remove the installspace.

    Advanced:
      Clean only specific parts of the workspace.

      -c, --cmake-cache     Clear the CMakeCache for each package, but leave build
                            and devel spaces.
      -o, --orphans         Remove only build directories whose source packages
                            are no longer enabled or in the source space. This
                            might require --force-cmake on the next build.
