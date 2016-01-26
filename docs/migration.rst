Migrating from catkin_make
==========================

Important Distinctions between ``catkin_make`` and ``catkin build``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unlike ``catkin_make``, the ``catkin`` command-line tool is not just a thin
wrapper around a ``CMake`` use pattern. The ``catkin build`` command builds
each package in a workspace's source space *in isolation* in order to prevent
buildtime cross-talk. As such, in its simplest use, ``catkin build`` is like a
parallelized version of ``catkin_make_isolated``. While there are many more
features in ``catkin_tools`` described in the rest of the documentation, this
chapter provides details on how to switch from using ``catkin_make`` or
``catkin_make_isolated``.

Operational Differences
^^^^^^^^^^^^^^^^^^^^^^^

- ``catkin_tools`` has no "top-level" ``CMakeLists.txt`` file. The **source
  space** simply contains a collection of packages. If you have been modifying
  this ``CMakeLists.txt`` file, those modifications will be ignored.
- Each package in a ``catkin_tools`` workspace has its own isolated build space.
- ``catkin build`` can be run from any directory under the workspace root
- ``catkin config`` stores many workspace configuration options which needed to
  be passed to each call of ``catkin_make``
- ``catkin build`` can build plain CMake packages if they have ``package.xml``
  files
- Packages built with ``catkin build`` can not access variables defined in
  other Catkin packages in the same workspace.
- Packages no longer need to define target dependencies on ROS messages built
  in other packages. All targets in a dependency are guaranteed to have been
  built before the current package.
- ``catkin_tools`` and ``catkin_make`` can use the same source space, but they
  must use different build, devel, and install spaces.

IDE Integration
^^^^^^^^^^^^^^^

Since all packages are built in isolation with ``catkin build``, you can't rely
on CMake's IDE integration to generate a single project for your entire workspace.

.. _migration-troubleshooting:

Migration Troubleshooting
^^^^^^^^^^^^^^^^^^^^^^^^^

When migrating from ``catkin_make`` to catkin build, the most common problems
come from Catkin packages taking advantge of package cross-talk in the CMake
configuration stage.

Many Catkin packages implicitly rely on other packages in a workspace to
declare and find dependencies. When switcing from ``catkin_make``, users
will often discover these bugs.

Common Issues
-------------

Unknown CMake command "catkin_package"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If ``find_package(catkin REQUIRED ...)`` isn't called, then the
``catkin_package()`` macro will not be available. If such a package builds with
``catkin_make``, it's because it's relying on another package in the same
workspace to do this work.

Compilation Errors (Missing Headers)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compilation errors can occur if required headers are not found. If
your package includes headers from ``${catkin_INCLUDE_DIRS}``, make sure *that*
package is finding the right Catkin packages in ``find_package(catkin
COMPONENTS ...)``.

If your package includes headers from other libraries, make sure those
libraries are found and those CMake variables are defined.

Linker Errors (Undefined References)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Linker errors are due to targets not being linked to required libraries. If
your target links against ``${catkin_LIBRARIES}``, make sure *that* package
is finding the right Catkin packages in ``find_package(catkin COMPONENTS ...)``.

If your target links against other libraries, make sure those libraries are 
found and those CMake variables are defined.

- https://github.com/catkin/catkin_tools/issues/228

Targets Not Being Built
~~~~~~~~~~~~~~~~~~~~~~~

It is critical for Catkin-based packages to call ``catkin_package()`` before
**any** targets are defined. Otherwise your targets will not be built into the
**devel space**. Previously with ``catkin_make``, as long as some package
called ``catkin_package()`` before your package was configured, the appropriate
target destinations were defined.

- https://github.com/catkin/catkin_tools/issues/220

Compiler Options Aren't Correct
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Your program might fail to build or fail to run due to incorrect compiler
options. Sometimes these compiler options are needed to use a dependency, but
aren't made available to the dependant package.

With ``catkin_make``, if a package sets certain compiler options, such as:

.. code-block:: cmake

    set(CMAKE_CXX_FLAGS "-std=c++ ${CMAKE_CXX_FLAGS}")

These options will be set for every package in the topological sort which is
built after it, even packages which don't depend on it.

With ``catkin build``, however, these effects are isolated, so even the packages
that need these options will not get them. The ``catkin_package()`` macro already
provides options for exporting libraries and include directories, but it does not
have an option for CMake variables.

To export such settings (or even execute code), the ``CFG_EXTRAS`` option
must be used with an accompanying CMake file. For more information on this option, 
see `the catkin_package() documentation <http://docs.ros.org/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package>`_.

- https://github.com/catkin/catkin_tools/issues/210
- https://github.com/carpe-noctem-cassel/cnc-msl/pull/1

Uncommon Issues
---------------

Exporting Build Utilities
~~~~~~~~~~~~~~~~~~~~~~~~~

Some Catkin packages provide build tools at configuration time, like scripts
for generating code or downloading resources from the internet. These
packages need to export absolute paths to such tools both when used in a
workspace and when installed.

For example, when using in a source space, the build tools from package
``my_build_util`` would be found at ``${CMAKE_CURRENT_SOURCE_DIR}/cmake``, but
when installed, they would be found in ``${my_build_util_DIR}``.

With ``catkin_make``, the path to these tools could be set to either the source
or install space in the provider package just by setting a CMake variable, which
would be "leaked" to all subsequently built packages.

With ``catkin build``, these paths need to be properly exported with
``CFG_EXTRAS``. A way to do this that works both out of a workspace and install
is shown below:

.. code-block:: cmake
    :caption: my_build_util-extras.cmake.em

    # generated from stdr_common/cmake/stdr_common-extras.cmake.em

    @[if DEVELSPACE]@
    # set path to source space
    set(my_build_util_EXTRAS_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/cmake")
    @[else]@
    # set path to installspace
    set(my_build_util_EXTRAS_DIR "${my_build_util_DIR}")
    @[end if]@


Exporting Non-Standard Library Output Locations or Prefixes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some users may choose to build library targets with non-standard output
locations or prefixes. However, the normal ``catkin_package()`` macro
cannot export libraries with such paths across packages.

Again, we can use the ``CFG_EXTRAS`` option to append the special library to
the ``${PROJECT_NAME}_LIBRARIES`` variable that ``catkin_package()`` exports to
other packages.

.. code-block:: cmake
    :caption: CMakeLists.txt

    catkin_package(
      ...
      LIBRARIES # NOTE: Not specified here, but in extras file
      CFG_EXTRAS my-extras.cmake
    )

    set_target_properties(
      ${PROJECT_NAME} PROPERTIES
      PREFIX ""
      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
    
.. code-block:: cmake
    :caption: my.cmake.in

    find_library(@PROJECT_NAME@_LIBRARY
                NAMES @PROJECT_NAME@
                PATHS "${@PROJECT_NAME@_DIR}/../../../@CATKIN_GLOBAL_LIB_DESTINATION@/"
                NO_DEFAULT_PATH)

    if(@PROJECT_NAME@_LIBRARY)
      # Multiple CMake projects case (i.e. 'catkin build'):
      # - The target has already been built when its dependencies require it
      # - Specify full path to found library
      list(APPEND @PROJECT_NAME@_LIBRARIES ${@PROJECT_NAME@_LIBRARY})
    else()
      # Single CMake project case (i.e. 'catkin_make'):
      # - The target has not been built when its dependencies require it
      # - Specify target name only
      list(APPEND @PROJECT_NAME@_LIBRARIES @PROJECT_NAME@)
    endif()


- https://github.com/catkin/catkin_tools/issues/128
- http://answers.ros.org/question/201036/how-can-catkin-find-ros-libraries-in-non-standard-locations/?answer=209923#post-id-209923


Controlling Python Version
~~~~~~~~~~~~~~~~~~~~~~~~~~

On some platforms, there are multiple versions of Python, and Catkin's
internal setup file generation might pick the wrong one. For ``catkin_make``, 
this is sometimes solved on a given platform by creating a shell alias which
sets the ``PYTHON_EXECUTABLE`` CMake variable.

For ``catkin build``, however, you can create a *verb alias* like the one
below, which overrides the default behavior of ``catkin build`` even in new
workspaces.

.. code-block:: yaml

  build: build -DPYTHON_EXECUTABLE=/usr/bin/python2.7

See :doc:`Verb Aliasing <advanced/verb_customization>` for more details.

- https://github.com/catkin/catkin_tools/issues/166

CLI Comparison with ``catkin_make`` and ``catkin_make_isolated``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Below are tables mapping ``catkin_make`` and ``catkin_make_isolated`` arguments
into ``catkin`` arguments.  Note that some ``catkin_make`` options can only be
achived with the ``catkin config`` verb.

=================================================  ============================================
 catkin_make ...                                    catkin ...                                 
=================================================  ============================================
 ``-C PATH``                                        ``-w PATH [build | config | ...]``         
-------------------------------------------------  --------------------------------------------
 ``--source PATH``                                  ``config --source-space PATH`` [1]_        
-------------------------------------------------  --------------------------------------------
 ``--build PATH``                                   ``config --build-space PATH`` [1]_         
-------------------------------------------------  --------------------------------------------
 ``--use-ninja``                                    *not yet available*                       
-------------------------------------------------  --------------------------------------------
 ``--force-cmake``                                  ``build --force-cmake``                    
-------------------------------------------------  --------------------------------------------
 ``--pkg PKG [PKG ...]``                            ``build --no-deps PKG [PKG ...]``          
-------------------------------------------------  --------------------------------------------
 ``--only-pkg-with-deps PKG [PKG ...]``             ``build PKG [PKG ...]``                    
-------------------------------------------------  --------------------------------------------
 ``--cmake-args ARG [ARG ...]``                     ``build --cmake-args ARG [ARG ...]`` [2]_
-------------------------------------------------  --------------------------------------------
 ``--make-args ARG [ARG ...]``                      ``build --make-args ARG [ARG ...]`` [2]_   
-------------------------------------------------  --------------------------------------------
 ``--override-build-tool-check``                    ``build --override-build-tool-check``      
-------------------------------------------------  --------------------------------------------
 ``ARG [ARG ...]``                                  ``build --make-args ARG [ARG ...]``        
-------------------------------------------------  --------------------------------------------
 ``install``                                        ``config --install`` [1]_   
-------------------------------------------------  --------------------------------------------
 ``-DCATKIN_DEVEL_PREFIX=PATH``                     ``config --devel-space PATH`` [1]_ 
-------------------------------------------------  --------------------------------------------
 ``-DCATKIN_INSTALL_PREFIX=PATH``                   ``config --install-space PATH`` [1]_ 
-------------------------------------------------  --------------------------------------------
 ``-DCATKIN_WHITELIST_PACKAGES="PKG[;PKG ...]"``    ``config --whitelist PKG [PKG ...]`` [1]_ 
=================================================  ============================================


========================================  ============================================
 catkin_make_isolated ...                  catkin ...
========================================  ============================================
 ``-C PATH``                               ``-w PATH [build | config | ...]``
----------------------------------------  --------------------------------------------
 ``--source PATH``                         ``config --source-space PATH`` [1]_
----------------------------------------  --------------------------------------------
 ``--build PATH``                          ``config --build-space PATH`` [1]_
----------------------------------------  --------------------------------------------
 ``--devel PATH``                          ``config --devel-space PATH`` [1]_
----------------------------------------  --------------------------------------------
 ``--merge``                               ``config --devel-layout merged`` [1]_
----------------------------------------  --------------------------------------------
 ``--install-space PATH``                  ``config --install-space PATH`` [1]_
----------------------------------------  --------------------------------------------
 ``--use-ninja``                           *not yet available*
----------------------------------------  --------------------------------------------
 ``--install``                             ``config --install`` [1]_
----------------------------------------  --------------------------------------------
 ``--force-cmake``                         ``build --force-cmake``
----------------------------------------  --------------------------------------------
 ``--no-color``                            ``build --no-color``
----------------------------------------  --------------------------------------------
 ``--pkg PKG [PKG ...]``                   ``build --no-deps PKG [PKG ...]``
----------------------------------------  --------------------------------------------
 ``--from-pkg PKG``                        ``build --start-with PKG``
----------------------------------------  --------------------------------------------
 ``--only-pkg-with-deps PKG [PKG ...]``    ``build PKG [PKG ...]``
----------------------------------------  --------------------------------------------
 ``--cmake-args ARG [ARG ...]``            ``build --cmake-args ARG [ARG ...]`` [2]_
----------------------------------------  --------------------------------------------
 ``--make-args ARG [ARG ...]``             ``build --make-args ARG [ARG ...]`` [2]_
----------------------------------------  --------------------------------------------
 ``--catkin-make-args ARG [ARG ...]``      ``build --catkin-make-args ARG [ARG ...]`` [2]_
----------------------------------------  --------------------------------------------
 ``--override-build-tool-check``           ``build --override-build-tool-check``
========================================  ============================================

.. [1] These options require a subsequent call to ``catkin build``, and the options
       will continue to persist until changed.
.. [2] These options, if passed to ``catkin build`` only affect that
       invocation. If passed to ``catkin config``, they will persist to
       subseqent calls to ``catkin build``.
