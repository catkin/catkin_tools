Adding New Build Types
======================

The current release of ``catkin_tools`` supports building two types of packages:

  - **Catkin** -- CMake packages that use the Catkin CMake macros
  - **CMake** -- "Plain" CMake packages

In order to fully support additional build types, numerous additions need to be made to the command-line interfaces so that the necessary parameters can be passed to the ``build`` verb.
For partial support, however, all that's needed is to add a build type identifier and a function for generating build jobs.

The supported build types are easily extendable using the ``setuptools`` ``entry_points`` interface without modifying the ``catkin_tools`` project, itself.
Regardless of what package the ``entry_point`` is defined in, it will be defined in the ``setup.py`` of that package, and will take this form: 

.. code-block:: python

    from setuptools import setup

    setup(
        ...
        entry_points={
            ...
            'catkin_tools.jobs': [
                'mybuild = my_package.some.module:description',
            ],
        },
    )

This entry in the ``setup.py`` places a file in the ``PYTHONPATH`` when either the ``install`` or the ``develop`` verb is given to ``setup.py``.
This file relates the key (in this case ``mybuild``) to a module and attribute (in this case ``my_package.some.module`` and ``description``).

Then the ``catkin`` command will use the ``pkg_resources`` modules to retrieve these mapping at run time.
Any entry for the ``catkin_tools.jobs`` group must point to a ``description`` attribute of a module, where the ``description`` attribute is a ``dict``.
The ``description`` ``dict`` should take this form:

.. code-block:: python

    description = dict(
        build_type='mybuild',
        description="Builds a package with the 'mybuild' build type",
        create_build_job=create_mybuild_build_job
    )

This ``dict`` defines all the information that the ``catkin`` command needs to create jobs for the ``mybuild`` build type.
The ``build_type`` key takes a string which is the build type identifier.
The ``description`` key takes a string which briefly describes the build type.
The ``create_build_job`` key takes a callable (function) factory which is called in order to create a ``Job`` to build a package of type ``mybuild``.

The signature of the factory callable should be similar to the following:

.. code-block:: python

    def create_mybuild_build_job(context, package, package_path, dependencies, **kwargs):
        # Initialize empty list of build stages
        stages = []

        # Add stages required to build ``mybuild``-type packages,
        # based on the configuration context.
        # ...

        # Create and return new build Job
        return Job(
            jid=package.name,
            deps=dependencies,
            stages=stages)

