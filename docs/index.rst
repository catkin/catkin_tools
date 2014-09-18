Catkin Command Line Tools
=========================

.. toctree::
   :hidden:
   :maxdepth: 2

   Installing <installing>
   quick_start
   mechanics
   config_summary
   cheat_sheet
   Troubleshooting <troubleshooting>
   verbs/catkin_build
   verbs/catkin_clean
   verbs/catkin_config
   verbs/catkin_create
   verbs/catkin_init
   verbs/catkin_list
   verbs/catkin_profile
   Advanced: Verb Aliasing <advanced/verb_customization>
   Advanced: Contributing Verbs <development/extending_the_catkin_command>
.. TODO: Advanced: Workspace Chaining <advanced/workspace_chaining>

This Python package provides command line tools for working with the catkin meta-buildsystem and catkin workspaces.

.. note::

  This is the documentation for the ``catkin`` command-line tool and **not**
  the Catkin package specification documentation. For documentation on writing
  catkin packages, see: http://docs.ros.org/api/catkin/html/

The ``catkin`` command
^^^^^^^^^^^^^^^^^^^^^^

The ``catkin`` Command-Line Interface (CLI) tool is the single point of entry
for most of the functionality provided by this package.
All invocations of the ``catkin`` CLI tool take this form:

.. code-block:: bash

    $ catkin [global options] <verb> [verb arguments and options]

The different capabilities of the ``catkin`` CLI tool are organized into
different sub-command "verbs." This is similar to common command-line tools
such as ``git`` or ``apt-get``.  Verbs include actions such as ``build`` which
builds a catkin workspace or ``list`` which simply lists the catkin packages
found in one or more folders.

Additionally, global options can be provided before the verb, options like
``-d`` for debug level verbosity or ``-h`` for help on the ``catkin`` CLI tool
itself.  Verbs can take arbitrary arguments and options, but they must all come
after the verb.  For more help on the usage of a particular verb, simply pass
the ``-h`` or ``--help`` option after the verb.

Built-in ``catkin`` command verbs
---------------------------------

Each of the following verbs is built-in to the ``catkin`` command and has its own detailed documentation:

- :doc:`build -- Build packages in a catkin workspace <verbs/catkin_build>`
- :doc:`config -- Configure a catkin workspace's layout and settings <verbs/catkin_config>`
- :doc:`clean -- Clean products generated in a catkin workspace <verbs/catkin_clean>`
- :doc:`create -- Create structrures like Catkin packages <verbs/catkin_create>`
- :doc:`init -- Initialize a catkin workspace <verbs/catkin_init>`
- :doc:`list -- Find and list information about catkin packages in a workspace <verbs/catkin_list>`
- :doc:`profile -- Manage different named configuration profiles <verbs/catkin_profile>`

Extending the ``catkin`` command
--------------------------------

If you would like to add a verb to the ``catkin`` command without modifying its source, please read :doc:`development/extending_the_catkin_command`.

