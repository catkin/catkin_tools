``catkin build`` -- Build Packages
==================================

The ``build`` verb is used to build one or more packages in a catkin workspace.
Like most verbs, ``build`` is context-aware and can be executed from within any directory contained by an initialized workspace.
If a workspace is not yet initialized, ``build`` can initialize it with the default configuration, but only if it is called from the workspace root.
Specific workspaces can also be built from arbitrary working directories  with the ``--workspace`` option.

.. note::

    To set up a workspace and clone the repositories used in the following examples, you can use `rosinstall_generator <http://wiki.ros.org/rosinstall_generator>`_ and `wstool <http://wiki.ros.org/wstool>`_.
    The following clones all of the ROS packages necessary for building the introductory ROS tutorials:

    .. literalinclude:: ../examples/ros_tutorials_ws/0_checkout.bash
        :language: bash

Basic Usage
^^^^^^^^^^^

Previewing The Build
--------------------

Before actually building anything in the workspace, it is useful to preview which packages will be built and in what order.
This can be done with the ``--dry-run`` option:

.. literalinclude:: ../examples/ros_tutorials_ws/2_dry_run.bash
   :language: bash

In addition to the listing the package names and in which order they would be built, it also displays the build type of each package.

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/alk9y0uhcymjwk6eg47s8tace.js" id="asciicast-alk9y0uhcymjwk6eg47s8tace" async></script></center>

Building a Workspace
--------------------

When no packages are given as arguments, ``catkin build`` builds the entire workspace.
It automatically creates directories for a **build space** and a **devel space**:

.. literalinclude:: ../examples/ros_tutorials_ws/3_build.bash
   :language: bash

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/cxq6cn449b8h0x7462kdy71jk.js" id="asciicast-cxq6cn449b8h0x7462kdy71jk" async></script></center>

After the build finishes, the **build space** contains directories containing the intermediate build products for each package, and the **devel space** contains an FHS layout into which all the final build products are written.

.. note::

    The products of ``catkin build`` differ significantly from the behavior of ``catkin_make``, for example, which would have all of the build files and intermediate build products in a combined **build space** or ``catkin_make_isolated`` which would have an isolated FHS directory for each package in the **devel space**.

Status Line
-----------

When running ``catkin build`` with default options, it displays a "live" status line similar to the following:

.. code-block:: none

    [build - 20.2] [18/34 complete] [4/4 jobs] [1 queued] [xmlrpcpp:make (66%) - 4.9] ...

The status line stays at the bottom of the screen and displays the continuously-updated progress of the entire build as well as the active build jobs which are still running. It is composed of the following information:

 * ``[build - <T>]`` -- The first block on the left indicates the total elapsed build time ``<T>`` in seconds thus far.
 * ``[<M>/<N> complete]``  --  The second block from the left indicates the build progress in terms of the number of completed packages, ``<M>`` out of the total number of packages to be built ``<N>``.
 * ``[<M>/<N> jobs]`` --  The third block from the left indicates the number of active total low-level jobs ``<M>`` out of the total number of low-level workers ``<N>``.
 * ``[<N> queued]`` --  The fourth block from the left indicates the number of jobs ``<N>`` whose dependencies have already been satisfied and are ready to be built.
 * ``[<N> failed]`` --  The fifth block from the left indicates the number of jobs ``<N>`` which have failed.
   This block only appears once one or more jobs has failed.
 * ``[<package>:<stage> (<P>%) - <T>]`` -- The remaining blocks show details on the active jobs.
   These include the percent complete, ``<P>``, of the stage, if available, as well as the time elapsed building the package, ``<T>``.

When necessary, the status line can be disabled by passing the ``--no-status`` option to ``catkin build``.
This is sometimes required when running ``catkin build`` from within a program that doesn't support the ASCII escape sequences required to reset and re-write the status line.

Console Messages
----------------

Normally, unless an error occurs, the output from each package's build process is collected but not printed to the console.
All that is printed is a pair of messages designating the start and end of a package's build.
This is formatted like the following for the ``genmsg`` package:

.. code-block:: none

    ...
    Starting  >>> {JOB}
    ...
    Finished  <<< {JOB}   [ {TIME} seconds ]
    ...

Error messages are printed whenever a build job writes to ``stderr``.
In such cases, the ``build`` verb will automatically print the captured ``stderr`` buffer under a ``Warnings`` header once the job has completed, similarly to below:

.. code-block:: none

    ____________________________________________________________________________
    Warnings   << {JOB}:{STAGE} {LOGFILE PATH}
    {WARNINGS}
    {REPRODUCTION COMMAND}
    ............................................................................
    Finished   << {JOB}            [ {TIME} seconds ]

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/1pumr3gcuxikbqyanikr6aznm.js" id="asciicast-1pumr3gcuxikbqyanikr6aznm" async></script></center>

Note that the first line displays the path to the interleaved log file, which persists until the build space is cleaned.
Additionally, if a package fails, the output to ``stderr`` is printed under the ``Errors`` header.

.. code-block:: none

    ____________________________________________________________________________
    Errors     << {JOB}:{STAGE} {LOGFILE PATH}
    {ERRORS}
    {REPRODUCTION COMMAND}
    ............................................................................
    Failed     << {JOB}:{STAGE}    [ Exited with code {EXIT CODE} ]
    Failed     << {JOB}            [ {TIME} seconds ]

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/8t1qsf2amqmm4p4n3jxdnuth5.js" id="asciicast-8t1qsf2amqmm4p4n3jxdnuth5" async></script></center>

All of the messages from the underlying jobs can be shown when using the ``-v`` or ``--verbose`` option.
This will print the normal messages when a build job starts and finishes as well as the interleaved output to ``stdout`` and ``stderr`` from each build command in a block.

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/9d9jklblvvyeq62y2r3ugpywp.js" id="asciicast-9d9jklblvvyeq62y2r3ugpywp" async></script></center>

All output can be printed interleaved with the ``--interleave-output`` option.
In this case, each line is prefixed with the job and stage from which it came.

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/3ocuvvztciubxay7nz41gri5z.js" id="asciicast-3ocuvvztciubxay7nz41gri5z" async></script></center>

Build Summary
-------------

At the end of each build, a brief build summary is printed to guarantee that anomalies aren't missed.
This summary displays the total run-time, the number of successful jobs, the number of jobs which produced warnings, and the number of jobs which weren't attempted due to failed dependencies.

.. code-block:: none

    [build] Runtime: 1.9 seconds total.
    [build] Summary: 4 of 7 jobs completed.
    [build]   Warnings:  None.
    [build]   Abandoned: 1 jobs were abandoned.
    [build]   Failed:    2 jobs failed.

A more detailed summary can also be printed with the ``--summarize`` command, which lists the result for each package in the workspace.

Building Subsets of Packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Consider a Catkin workspace with a **source space** populated with the following Catkin packages which have yet to be built:

.. code-block:: bash

    $ pwd
    /tmp/path/to/my_catkin_ws

    $ ls ./*
    ./src:
    catkin             console_bridge     genlisp            genpy
    message_runtime    ros_comm           roscpp_core        std_msgs
    common_msgs        gencpp             genmsg             message_generation
    ros                ros_tutorials      rospack


Building Specific Packages
--------------------------

Specific packages can also be built by specifying them as positional arguments after the ``build`` verb:

.. literalinclude:: ../examples/ros_tutorials_ws/6_build_partial.bash
   :language: bash

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/7oij906zndo56g64slix0lhv0.js" id="asciicast-7oij906zndo56g64slix0lhv0" async></script></center>

As shown above, only 4 packages (``roslib`` and its dependencies), of the total 36 packages would be built.

Context-Aware Building
----------------------

In addition to building all packages or specified packages with various dependency requirements, ``catkin build`` can also determine the package containing the current working directory.
This is equivalent to specifying the name of the package on the command line, and is done by passing the ``--this`` option to ``catkin build`` like the following:

.. literalinclude:: ../examples/ros_tutorials_ws/7_build_this.bash
   :language: bash

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/8bgk2alfs1srz58xknchidy9d.js" id="asciicast-8bgk2alfs1srz58xknchidy9d" async></script></center>

Skipping Packages
-----------------

Suppose you built every package up to ``roslib``, but that package had a build error.
After fixing the error, you could run the same build command again, but the ``build`` verb provides an option to save time in this situation.
If re-started from the beginning, none of the products of the dependencies of ``roslib`` would be re-built, but it would still take some time for the underlying build system to verify that for each package.

Those checks could be skipped, however, by jumping directly to a given package.
You could use the ``--start-with`` option to continue the build where you left off after fixing the problem.

.. literalinclude:: ../examples/ros_tutorials_ws/8_build_start_with.bash
   :language: bash

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/81cmmsatgv7am8pn5k61imvku.js" id="asciicast-81cmmsatgv7am8pn5k61imvku" async></script></center>

.. note::

  ``catkin build`` will assume that all dependencies leading up to the package
  specified with the ``--start-with`` option have already been successfully
  built.

Building Single Packages
------------------------

If you're only interested in building a *single* package in a workspace, you can also use the ``--no-deps`` option along with a package name.
This will skip all of the package's dependencies, build the given package, and then exit.

.. literalinclude:: ../examples/ros_tutorials_ws/9_build_no_deps.bash
   :language: bash

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/1uop75vi9bs75ikthtisyi34p.js" id="asciicast-1uop75vi9bs75ikthtisyi34p" async></script></center>

Building and Running Tests
^^^^^^^^^^^^^^^^^^^^^^^^^^

Running tests for a given package typically is done by invoking a special ``make`` target like ``test`` or ``run_tests``.
catkin packages all define the ``run_tests`` target which aggregates all types of tests and runs them together.
So in order to get tests to build and run for your packages you need to pass them this additional ``run_tests`` or ``test`` target as a command line option to ``make``.

To run catkin tests for all catkin packages in the workspace, use the following:

.. code-block:: bash

    $ catkin run_tests

Or the longer version:

.. code-block:: bash

    $ catkin build [...] --catkin-make-args run_tests

To run a catkin test for a specific catkin package, from a directory within that package:

.. code-block:: bash

    $ catkin run_tests --no-deps --this

For non-catkin packages which define a ``test`` target, you can do this:

.. code-block:: bash

    $ catkin build [...] --make-args test

If you want to run tests for just one package, then you should build that package and this narrow down the build to just that package with the additional make argument:

.. code-block:: bash

    $ # First build the package
    $ catkin build package
    ...
    $ # Then run its tests
    $ catkin build package --no-deps --catkin-make-args run_tests
    $ # Or for non-catkin packages
    $ catkin build package --no-deps --make-args test

For catkin packages and the ``run_tests`` target, failing tests will not result in an non-zero exit code.
So if you want to check for failing tests, use the ``catkin_test_results`` command like this:

.. code-block:: bash

    $ catkin_test_results build/<package name>

The result code will be non-zero unless all tests passed.


Advanced Options
^^^^^^^^^^^^^^^^

Temporarily Changing Build Flags
--------------------------------

While the build configuration flags are set and stored in the build context, it's possible to temporarily override or augment them when using the ``build`` verb.

.. code-block:: bash

    $ catkin build --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter"

Building With Warnings
----------------------

It can sometimes be useful to compile with additional warnings enabled across your whole catkin workspace.
To achieve this, use a command similar to this:

.. code-block:: bash

    $ catkin build -v --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter"

This command passes the ``-DCMAKE_C_FLAGS=...`` argument to all invocations of ``cmake``.


Configuring Build Jobs
----------------------

By default ``catkin build`` on a computer with ``N`` cores will build up to ``N`` packages in parallel and will distribute ``N`` ``make`` jobs among them using an internal job server.
If your platform doesn't support job server scheduling, ``catkin build`` will pass ``-jN -lN`` to ``make`` for each package.

You can control the maximum number of packages allowed to build in parallel by using the ``-p`` or ``--parallel-packages`` option and you can change the number of ``make`` jobs available with the ``-j`` or ``--jobs`` option.

By default, these jobs options aren't passed to the underlying ``make`` command.
To disable the job server, you can use the ``--no-jobserver`` option, and you can pass flags directly to ``make`` with the ``--make-args`` option.

.. note::

    Jobs flags (``-jN`` and/or ``-lN``) can be passed directly to ``make`` by
    giving them to ``catkin build``, but other ``make`` arguments need to be
    passed to the ``--make-args`` option.


Configuring Memory Use
----------------------

In addition to CPU and load limits, ``catkin build`` can also limit the number of running jobs based on the available memory, using the hidden ``--mem-limit`` flag.
This flag requires installing the Python ``psutil`` module and is useful on systems without swap partitions or other situations where memory use needs to be limited.

Memory is specified either by percent or by the number of bytes.

For example, to specify that ``catkin build`` should not start additional parallel jobs when 50% of the available memory is used, you could run:

.. code-block:: bash

    $ catkin build --mem-limit 50%

Alternatively, if it should not start additional jobs when over 4GB of memory is used, you can specify:

.. code-block:: bash

    $ catkin build --mem-limit 4G


Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_build.txt
   :language: text
