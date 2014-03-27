Catkin Command Line Tools
=========================

This Python package provides command line tools for working with catkin and catkin workspaces.

The ``catkin`` command
----------------------

The ``catkin`` Command-Line Interface (CLI) tool is the single point of entry for most of the functionality provided by this package.
All invocations of the ``catkin`` CLI tool take this form:

.. code-block:: bash

    $ catkin [global options] <verb> [verb arguments and options]

The ``catkin`` CLI tool requires that you provide a verb.
The verbs could be many things, like ``build`` which builds a catkin workspace or ``list`` which simply lists the catkin packages found in one or more folders.
Optionally, global options can be provided before the verb, things like ``-d`` for debug level verbosity or ``-h`` for help on the ``catkin`` CLI tool itself.
Verbs can take arbitrary arguments and options, but they must all come after the verb.
For more help on a particular verb, simply pass ``-h`` or ``--help`` after the verb.

Verb aliasing
^^^^^^^^^^^^^

The ``catkin`` command allows you to create verb "aliases" and it also comes with some defaults.
Verb aliases are defined in YAML files in the ``verb_aliases`` folder in the catkin config folder.
The catkin config folder is located at ``~/.config/catkin``, therefore the alias files are located in ``~/.config/catkin/verb_aliases``.
By default there is one file in that directory called ``00-default-aliases.yaml``, but any files in that folder which end with either ``.yaml`` or ``.yml`` will be processed.
The built-in file should not be edited, because it is kept up-to-date by the ``catkin`` command.

Verb alias files are processed in ls list order, so making files which start with larger numbers will override files with smaller numbers.
In this way you can override the built-in aliases using a file which starts with a higher number.
For example, the ``install: build --install`` alias exists in the default file, but you can create this file to override it (``~/.config/catkin/verb_aliases/01-my-aliases.yml``):

.. code-block:: yaml

    install: build --install --merge-devel

You can also nullify or unset aliases by setting their values to ``null``.
So, for example, the ``ls: list`` alias is defined in the default aliases, you can override it with this entry in a custom file:

.. code-block:: yaml

    ls: null

You can list the available aliases using the ``--list-aliases`` option to the ``catkin`` command:

.. code-block:: bash

    $ catkin --list-aliases
    i: install
    b: build
    ls: list
    install: build --install

Additionally, verb aliases can be recursive, for instance the ``i`` alias expands to ``install`` and that alias in turn expands to ``build --install``.
The ``catkin`` command shows the expansion of aliases so that the behavior is more transparent:

.. code-block:: bash

    $ catkin i
    ==> Expanding alias 'i' from 'catkin i' to 'catkin install'
    ==> Expanding alias 'install' from 'catkin install' to 'catkin build --install'
    ...

Built-in ``catkin`` command verbs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

    build: Verb for building a catkin workspace <commands/catkin_build>
    list: Verb for finding and listing information about catkin packages <commands/catkin_list>

Installing
----------

You can install the ``catkin_tools`` package as a binary through a package manager like ``pip`` or ``apt-get``, or from source.

Installing on Ubuntu with apt-get
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First you must have the ROS repositories which contain the ``.deb`` for ``catkin_tools``:

.. code-block:: bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

Once you have added that repository, run these commands to install ``catkin_tools``:

.. code-block:: bash

    $ sudo apt-get update
    $ sudo apt-get install python-catkin-tools

Installing on other platforms with pip
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Simply install it with ``pip``:

.. code-block:: bash

    $ sudo pip install -U catkin_tools

Installing from source
^^^^^^^^^^^^^^^^^^^^^^

First clone the source for ``catkin_tools``:

.. code-block:: bash

    $ git clone https://github.com/catkin/catkin_tools.git
    $ cd catkin_tools

Then install with the ``setup.py`` file:

.. code-block:: bash

    $ python setup.py install

Developing
----------

Listed here are some useful tips for developing against ``catkin_tools``.

Install ``catkin_tools`` for developing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To setup ``catkin_tools`` for fast iteration during development, use the ``develop`` verb to ``setup.py``:

.. code-block:: bash

    $ python setup.py develop

Now the commands, like ``catkin``, will be in the system path and the local source files located in the ``catkin_tools`` folder will be on the ``PYTHONPATH``. When you are done with your development, undo this by running this command:

.. code-block:: bash

    $ python setup.py develop -u
