Verb Aliasing
=============

The ``catkin`` command allows you to define your own verb "aliases" which expand to more complex expressions including built-in verbs, command-line options, and other verb aliases.
These are processed before any other command-line processing takes place, and can be useful for making certain use patterns more convenient.

The Built-In Aliases
^^^^^^^^^^^^^^^^^^^^

You can list the available aliases using the ``--list-aliases`` option to the ``catkin`` command.
Below are the built-in aliases as displayed by this command: 

.. code-block:: bash

    $ catkin --list-aliases
    b: build
    bt: b --this
    ls: list
    install: config --install


Defining Additional Aliases
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Verb aliases are defined in the ``verb_aliases`` sub-directory of the catkin config folder, ``~/.config/catkin/verb_aliases``.
Any YAML files in that folder (files with a ``.yaml`` extension) will be processed as definition files.

These files are formatted as simple YAML dictionaries which map aliases to expanded expressions, which must be composed of other ``catkin`` verbs, options, or aliases: 

.. code-block:: yaml

  <ALIAS>: <EXPRESSION>

For example, aliases which configure a workspace profile so that it ignores the value of the ``CMAKE_PREFIX_PATH`` environment variable, and instead *extends* one or another ROS install spaces could be defined as follows: 

.. code-block:: yaml

  # ~/.config/catkin/verb_aliases/10-ros-distro-aliases.yaml
  extend-sys: config --profile sys --extend /opt/ros/indigo -x _sys
  extend-overlay: config --profile overlay --extend ~/ros/indigo/install -x _overlay

After defining these aliases, one could use them with optional additional options and build a given configuration profile.

.. code-block:: bash

  $ catkin extend-overlay
  $ catkin profile set overlay
  $ catkin build some_package

.. note::

  The ``catkin`` command will initialize the ``verb_aliases`` directory with a   file named ``00-default-aliases.yaml`` containing the set of built-in   aliases.
  These defaults can be overridden by adding additional definition   files, but the default alias file should not be modified since any changes to   it will be over-written by invocations of the ``catkin`` command.

Alias Precedence and Overriding Aliases
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Verb alias files in the ``verb_aliases`` directory are processed in alphabetical order, so files which start with larger numbers will override files with smaller numbers.
In this way you can override the built-in aliases using a file which starts with a number higher than ``00-``.

For example, the ``bt: build --this`` alias exists in the default alias file, ``00-default-aliases.yaml``, but you can create a file to override it with an alternate definition defined in a file named ``01-my-aliases.yaml``.

.. code-block:: yaml

    # ~/.config/catkin/verb_aliases/01-my-aliases.yaml
    # Override `bt` to build with no deps
    bt: build --this --no-deps

You can also disable or unset an alias by setting its value to ``null``.
For example, the ``ls: list`` alias is defined in the default aliases, but you can override it with this entry in a custom file named something like ``02-unset.yaml``: 

.. code-block:: yaml

    # ~/.config/catkin/verb_aliases/02-unset.yaml
    # Disable `ls` alias
    ls: null

Recursive Alias Expansion
^^^^^^^^^^^^^^^^^^^^^^^^^

Additionally, verb aliases can be recursive, for instance in the ``bt`` alias, the ``b`` alias expands to ``build`` so that ``b --this`` expands to ``build --this``.
The ``catkin`` command shows the expansion of aliases when they are invoked so that their behavior is more transparent: 

.. code-block:: bash

    $ catkin bt
    ==> Expanding alias 'bt' from 'catkin bt' to 'catkin b --this'
    ==> Expanding alias 'b' from 'catkin b --this' to 'catkin build --this'
    ...

