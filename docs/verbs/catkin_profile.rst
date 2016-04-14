``catkin profile`` -- Manage Profiles
=====================================

Many verbs contain a ``--profile`` option, which selects which configuration profile to use, without which it will use the "active" profile.
The ``profile`` verb enables you to manager the available profiles as well as set the "active" profile when using other verbs.

Even without using the ``profile`` verb, any use of the ``catkin`` command which changes the workspace is implicitly using a configuration profile called ``default``.

The ``profile`` verb has several sub-commands for profile management.
These include the following:

- ``list`` -- List the available profiles
- ``set`` -- Set the active profile by name.
- ``add`` -- Add a new profile by name.
- ``rename`` -- Rename a given profile.
- ``remove`` -- Remove a profile by name.

Creating Profiles Automatically
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After initializing a workspace, you can start querying information about profiles.
Until you execute a verb which actually writes a profile configuration, however, there will be no profiles listed:

.. code-block:: bash

    $ mkdir -p /tmp/path/to/my_catkin_ws/src
    $ cd /tmp/path/to/my_catkin_ws
    $ catkin init
    $ catkin profile list
    [profile] This workspace has no metadata profiles. Any configuration
    settings will automatically by applied to a new profile called `default`.

To see these effects, you can run ``catkin config`` to write a default configuration to the workspace:

.. code-block:: bash

    $ cd /tmp/path/to/my_catkin_ws
    $ catkin config
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
    Workspace:                   /tmp/path/to/my_catkin_ws
    Source Space:       [exists] /tmp/path/to/my_catkin_ws/src
    Build Space:       [missing] /tmp/path/to/my_catkin_ws/build
    Devel Space:       [missing] /tmp/path/to/my_catkin_ws/devel
    Install Space:     [missing] /tmp/path/to/my_catkin_ws/install
    DESTDIR:                     None
    --------------------------------------------------------------
    Isolate Develspaces:         False
    Install Packages:            False
    Isolate Installs:            False
    --------------------------------------------------------------
    Additional CMake Args:       None
    Additional Make Args:        None
    Additional catkin Make Args: None
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------
    $ catkin profile list
    [profile] Available profiles:
    - default (active)

The ``profile`` verb now shows that the profile named "default" is available and is active.
Calling ``catkin config`` with the ``--profile`` argument will automatically create a profile based on the given configuration options:

.. code-block:: bash

    $ catkin config --profile alternate -x _alt
    ------------------------------------------------------------------
    Profile:                     alternate
    Extending:                   None
    Workspace:                   /tmp/path/to/my_catkin_ws
    Source Space:       [exists] /tmp/path/to/my_catkin_ws/src
    Build Space:       [missing] /tmp/path/to/my_catkin_ws/build_alt
    Devel Space:       [missing] /tmp/path/to/my_catkin_ws/devel_alt
    Install Space:     [missing] /tmp/path/to/my_catkin_ws/install_alt
    DESTDIR:                     None
    ------------------------------------------------------------------
    Isolate Develspaces:         False
    Install Packages:            False
    Isolate Installs:            False
    ------------------------------------------------------------------
    Additional CMake Args:       None
    Additional Make Args:        None
    Additional catkin Make Args: None
    ------------------------------------------------------------------
    Workspace configuration appears valid.
    ------------------------------------------------------------------
    $ catkin profile list
    [profile] Available profiles:
    - alternate
    - default (active)

Note that while the profile named ``alternate`` has been configured, it is still not *active*, so any calls to catkin-verbs without an explicit ``--profile alternate`` option will still use the profile named ``default``.

Explicitly Creating Profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Profiles can also be added explicitly with the ``add`` command.
This profile can be initialized with configuration information from either the default settings or another profile.

.. code-block:: bash

    $ catkin profile list
    [profile] Available profiles:
    - alternate
    - default (active)
    $ catkin profile add alternate_2 --copy alternate
    [profile] Created a new profile named alternate_2 based on profile alternate
    [profile] Available profiles:
    - alternate
    - alternate_2
    - default (active)

Setting the Active Profile
^^^^^^^^^^^^^^^^^^^^^^^^^^

The active profile can be easily set with the ``set`` sub-command.
Suppose a workspace has the following profiles:

.. code-block:: bash

    $ catkin profile list
    [profile] Available profiles:
    - alternate
    - alternate_2
    - default (active)
    $ catkin profile set alternate_2
    [profile] Activated catkin metadata profile: alternate_2
    [profile] Available profiles:
    - alternate
    - alternate_2 (active)
    - default

Renaming and Removing Profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``profile`` verb can also be used for renaming and removing profiles:

.. code-block:: bash

    $ catkin profile list
    [profile] Available profiles:
    - alternate
    - alternate_2 (active)
    - default
    $ catkin profile rename alternate_2 alternate2
    [profile] Renamed profile alternate_2 to alternate2
    [profile] Available profiles:
    - alternate
    - alternate2 (active)
    - default
    $ catkin profile remove alterate
    [profile] Removed profile: alternate
    [profile] Available profiles:
    - alternate2 (active)
    - default

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_profile.txt
   :language: text

``catkin profile list``
-----------------------

.. literalinclude:: cli/catkin_profile_list.txt
   :language: text

``catkin profile set``
-----------------------

.. literalinclude:: cli/catkin_profile_set.txt
   :language: text

``catkin profile add``
-----------------------

.. literalinclude:: cli/catkin_profile_add.txt
   :language: text

``catkin profile rename``
-------------------------

.. literalinclude:: cli/catkin_profile_rename.txt
   :language: text

``catkin profile remove``
-------------------------

.. literalinclude:: cli/catkin_profile_remove.txt
   :language: text

