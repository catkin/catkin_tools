``catkin test`` -- Test Packages
==================================

The ``test`` verb is used to test one or more packages in a catkin workspace.
Like most verbs, ``test`` is context-aware and can be executed from within any directory contained by an initialized workspace.
Specific workspaces can also be built from arbitrary working directories  with the ``--workspace`` option.

Basic Usage
^^^^^^^^^^^

Before running tests for packages in the workspace, they have to be built with ``catkin build``.
Then, to run the tests, use the following:

.. code-block:: bash

    $ catkin test

Under the hood, this invokes the ``make`` targets ``run_tests`` or ``test``, depending on the package.
catkin packages all define the ``run_tests`` target which aggregates all types of tests and runs them together.
For cmake packages that do not use catkin, the ``test`` target is invoked.
This target is usually populated by cmake when the ``enable_testing()`` command is used in the ``CMakeLists.txt``.
If it does not exist, a warning is printed.

To run a catkin test for a specific catkin package, from a directory within that package:

.. code-block:: bash

    $ catkin test --this

Advanced Options
^^^^^^^^^^^^^^^^

To manually specify a different ``make`` target, use ``--test-target``:

.. code-block:: bash

    $ catkin test --test-target gtest

It is also possible to use ``--catkin-test-target`` to change the target only for catkin packages.

Normally, the tests are run in parallel, similar to the build jobs of ``catkin build``.
To avoid building packages in parallel or to reduce the amount of parallel jobs, use ``-p``:

.. code-block:: bash

    $ catkin test -p 1

Sometimes, it can be helpful to see the output of tests while they are still running.
This can be achieved using ``--interleave-output``.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_test.txt
   :language: text
