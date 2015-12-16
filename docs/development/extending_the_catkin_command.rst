Extending the ``catkin`` command
================================

The ``catkin`` command is designed to be easily extendable using the ``setuptools`` ``entry_points`` interface without modifying the ``catkin_tools`` project, itself.
Regardless of what package the ``entry_point`` is defined in, it will be defined in the ``setup.py`` of that package, and will take this form:

.. code-block:: python

    from setuptools import setup

    setup(
        ...
        entry_points={
            ...
            'catkin_tools.commands.catkin.verbs': [
                # Example from catkin_tools' setup.py:
                # 'list = catkin_tools.verbs.catkin_list:description',
                'my_verb = my_package.some.module:description',
            ],
        },
    )

This entry in the ``setup.py`` places a file in the ``PYTHONPATH`` when either the ``install`` or the ``develop`` verb is given to ``setup.py``.
This file relates the key (in this case ``my_verb``) to a module and attribute (in this case ``my_package.some.module`` and ``description``).
Then the ``catkin`` command will use the ``pkg_resources`` modules to retrieve these mapping at run time.
Any entry for the ``catkin_tools.commands.catkin.verbs`` group must point to a ``description`` attribute of a module, where the ``description`` attribute is a ``dict``.
The ``description`` ``dict`` should take this form (the description from the ``build`` verb for example):

.. code-block:: python

    description = dict(
        verb='build',
        description="Builds a catkin workspace",
        main=main,
        prepare_arguments=prepare_arguments,
        argument_preprocessor=argument_preprocessor,
    )

This ``dict`` defines all the information that the ``catkin`` command needs to provide and execute your verb.
The ``verb`` key takes a string which is the verb name (as shown in help and used for invoking the verb).
The ``description`` key takes a string which is the description which is shown in the ``catkin -h`` output.
The ``main`` key takes a callable (function) which is called when the verb is invoked.
The signature of the main callable should be like this:

.. code-block:: python

    def main(opts):
        # ...
        return 0

Where the ``opts`` parameter is the ``Namespace`` object returns from ``ArgumentParser.parse_args(...)`` and should return an exit code which is passed to ``sys.exit``.

The ``prepare_arguments`` key takes a function with this signature:

.. code-block:: python

    def prepare_arguments(parser):
        add = parser.add_argument
        # What packages to build
        add('packages', nargs='*',
            help='Workspace packages to build, package dependencies are built as well unless --no-deps is used. '
                 'If no packages are given, then all the packages are built.')
        add('--no-deps', action='store_true', default=False,
            help='Only build specified packages, not their dependencies.')

        return parser

The above example is a snippet from the ``build`` verb's ``prepare_arguments`` function.
The purpose of this function is to take a given ``ArgumentParser`` object, which was created by the ``catkin`` command, and add this verb's ``argparse`` arguments to it and then return it.

Finally, the ``argument_preprocessor`` command is an optional entry in the ``description`` ``dict`` which has this signature:

.. code-block:: python

    def argument_preprocessor(args):
        """Processes the arguments for the build verb, before being passed to argparse"""
        # CMake/make pass-through flags collect dashed options. They require special
        # handling or argparse will complain about unrecognized options.
        args = sys.argv[1:] if args is None else args
        extract_make_args = extract_cmake_and_make_and_catkin_make_arguments
        args, cmake_args, make_args, catkin_make_args = extract_make_args(args)
        # Extract make jobs flags.
        jobs_flags = extract_jobs_flags(' '.join(args))
        if jobs_flags:
            args = re.sub(jobs_flags, '', ' '.join(args)).split()
            jobs_flags = jobs_flags.split()
        extras = {
            'cmake_args': cmake_args,
            'make_args': make_args + (jobs_flags or []),
            'catkin_make_args': catkin_make_args,
        }
        return args, extras

The above example is the ``argument_preprocessor`` function for the ``build`` verb.
The purpose of the ``argument_preprocessor`` callable is to allow the verb to preprocess its own arguments before they are passed to ``argparse``.
In the case of the ``build`` verb, it is extracting the CMake and Make arguments before having them passed to ``argparse``.
The input parameter to this function is the list of arguments which come after the verb, and this function is only called when this verb has been detected as the first positional argument to the ``catkin`` command.
So, you do not need to worry about making sure the arguments you just got are yours.
This function should return a tuple where the first item in the tuple is the potentially modified list of arguments, and the second item is a dictionary of keys and values which should be added as attributes to the ``opts`` parameter which is later passed to the ``main`` callable.
In this way you can take the arguments for your verb, parse them, remove some, add some or whatever, then you can additionally return extra information which needs to get passed around the ``argparse`` ``parse_args`` function.
Most verbs should not need to do this, and in fact the built-in ``list`` verb's ``description`` ``dict`` does not include one:

.. code-block:: python

    description = dict(
        verb='list',
        description="Lists catkin packages in a given folder",
        main=main,
        prepare_arguments=prepare_arguments,
    )

Hopefully, this information will help you get started when you want to extend the ``catkin`` command with custom verbs.
