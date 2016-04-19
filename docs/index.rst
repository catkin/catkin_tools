Catkin Command Line Tools
=========================

.. toctree::
   :name: tocmain
   :caption: Main Overview
   :glob:
   :hidden:
   :maxdepth: 2

   Installing <installing>
   History <history>
   quick_start
   cheat_sheet
   Migration Guide <migration>
   mechanics
   build_types
   Troubleshooting <troubleshooting>

.. toctree::
   :name: tocverbs
   :caption: Verb Details
   :glob:
   :hidden:
   :maxdepth: 2

   verbs/*

.. toctree::
   :name: tocadv
   :caption: Advanced Usage
   :glob:
   :hidden:
   :maxdepth: 2

   Shell Support <advanced/catkin_shell_verbs>
   Verb Aliasing <advanced/verb_customization>
.. Workspace Chaining <advanced/workspace_chaining>

.. toctree::
   :name: tocdes
   :caption: Design
   :glob:
   :hidden:
   :maxdepth: 2

   Linked Devel Space <advanced/linked_develspace>
   Execution Engine <advanced/job_executor>

.. toctree::
   :name: toccontrib
   :caption: Contributing
   :glob:
   :hidden:
   :maxdepth: 2

   development/adding_build_types
   Adding New Verbs <development/extending_the_catkin_command>

This Python package provides command line tools for working with the catkin meta-buildsystem and catkin workspaces.
These tools are separate from the Catkin CMake macros used in Catkin source packages.
For documentation on creating catkin packages, see: http://docs.ros.org/api/catkin/html/

.. note::

  This package was announced in March 2015 and is still in beta. See the `GitHub Milestones <https://github.com/catkin/catkin_tools/milestones>`_ for the current release schedule and roadmap.

.. note::

  Users of ``catkin_make`` and ``catkin_make_isolated`` should go to the
  :doc:`Migration Guide <migration>` for help transitioning to ``catkin
  build``.

The ``catkin`` Command
^^^^^^^^^^^^^^^^^^^^^^

.. .. raw:: html
..
..   <script type="text/javascript" src="https://asciinema.org/a/1lsef4d23r6hxh5kn8vjcx0el.js" id="asciicast-1lsef4d23r6hxh5kn8vjcx0el" async></script>

The ``catkin`` Command-Line Interface (CLI) tool is the single point of entry for most of the functionality provided by this package.
All invocations of the ``catkin`` CLI tool take this form:

.. code-block:: bash

    $ catkin [global options] <verb> [verb arguments and options]

The different capabilities of the ``catkin`` CLI tool are organized into different sub-command "verbs." This is similar to common command-line tools such as ``git`` or ``apt-get``.
Verbs include actions such as ``build`` which builds a catkin workspace or ``list`` which simply lists the catkin packages found in one or more folders.

Verbs can take arbitrary arguments and options, but they must all come after the verb.
For more help on the usage of a particular verb, simply pass the ``-h`` or ``--help`` option after the verb.

Built-in ``catkin`` Verbs
-------------------------

Each of the following verbs is built-in to the ``catkin`` command and has its own detailed documentation:

- :doc:`build -- Build packages in a catkin workspace <verbs/catkin_build>`
- :doc:`config -- Configure a catkin workspace's layout and settings <verbs/catkin_config>`
- :doc:`clean -- Clean products generated in a catkin workspace <verbs/catkin_clean>`
- :doc:`create -- Create structures like Catkin packages <verbs/catkin_create>`
- :doc:`env -- Run commands with a modified environemnt <verbs/catkin_env>`
- :doc:`init -- Initialize a catkin workspace <verbs/catkin_init>`
- :doc:`list -- Find and list information about catkin packages in a workspace <verbs/catkin_list>`
- :doc:`locate -- Get important workspace directory paths <verbs/catkin_locate>`
- :doc:`profile -- Manage different named configuration profiles <verbs/catkin_profile>`

Contributed Third Party Verbs
-----------------------------

- `lint -- Check catkin packages for common errors <https://github.com/fkie/catkin_lint>`_

Shell Support for the ``catkin`` Command
----------------------------------------

If you are using ``bash`` or ``zsh``, then you can source an extra setup file to gain access to some additional verbs.
For more information see: :doc:`advanced/catkin_shell_verbs`.

Extending the ``catkin`` command
--------------------------------

If you would like to add a verb to the ``catkin`` command without modifying its source, please read :doc:`Adding New Verbs <development/extending_the_catkin_command>`.
