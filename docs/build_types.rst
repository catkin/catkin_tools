Supported Build Types
=====================

The current release of ``catkin_tools`` supports building two types of packages:

  - **Catkin** -- CMake packages that use the Catkin CMake macros
  - **CMake** -- "Plain" CMake packages

There is currently limited support for adding other build types.
For information on extending ``catkin_tools`` to be able to build other types of packages, see :doc:`Adding New Build Types <development/adding_build_types>`.
Below are details on the stages involved in building a given package for each of the currently-supported build types.

Catkin
^^^^^^

Catkin packages are CMake packages which utilize the Catkin CMake macros for finding packages and defining configuration files.

Configuration Arguments
-----------------------

  - ``--cmake-args``
  - ``--make-args``
  - ``--catkin-make-args``

Build Stages
------------

==============  ============  ==================================================
 First           Subsequent    Description
==============  ============  ==================================================
 ``mkdir``                    | Create package build space if it doesn't exist.
----------------------------  --------------------------------------------------
 ``cmake``       ``check``    | Run CMake configure step **once** for the
                              | first build and the ``cmake_check_build_system``
                              | target for subsequent builds unless the
                              | ``--force-cmake`` argument is given.
--------------  ------------  --------------------------------------------------
 ``preclean`` `optional`      | Run the ``clean`` target before building.
                              | This is only done with the ``--pre-clean`` \
                                option.
----------------------------  --------------------------------------------------
 ``make``                     | Build the default target with GNU make.
----------------------------  --------------------------------------------------
 ``install`` `optional`       | Run the ``install`` target after building.
                              | This is only done with the ``--install`` option.
----------------------------  --------------------------------------------------
 ``setupgen``                 | Generate a ``setup.sh`` file to "source" the \
                              | result space.
----------------------------  --------------------------------------------------
 ``envgen``                   | Generate an ``env.sh`` file for loading the \
                              | result space's environment.
============================  ==================================================

CMake
^^^^^

Configuration Arguments
-----------------------

  - ``--cmake-args``
  - ``--make-args``

Build Stages
------------

==============  ============  ==================================================
 First           Subsequent    Description
==============  ============  ==================================================
 ``mkdir``                    | Create package build space if it doesn't exist.
----------------------------  --------------------------------------------------
 ``cmake``       ``check``    | Run CMake configure step **once** for the
                              | first build and the ``cmake_check_build_system``
                              | target for subsequent builds unless the
                              | ``--force-cmake`` argument is given.
--------------  ------------  --------------------------------------------------
 ``preclean`` `optional`      | Run the ``clean`` target before building.
                              | This is only done with the ``--pre-clean`` \
                                option.
----------------------------  --------------------------------------------------
 ``make``                     | Build the default target with GNU make.
----------------------------  --------------------------------------------------
 ``install``                  | Run the ``install`` target after building,
                              | and install products to the **devel space**.
                              | If the ``--install`` option is given,
                              | products are installed to the \
                                **install space** instead.
----------------------------  --------------------------------------------------
 ``setupgen``                 | Generate a ``setup.sh`` file if necessary.
============================  ==================================================


