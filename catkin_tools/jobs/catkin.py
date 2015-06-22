# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import csv
import errno
import glob
import os
import stat
import sys
import threading

from multiprocessing import cpu_count

from catkin_tools.argument_parsing import handle_make_arguments

from catkin_tools.common import mkdir_p
from catkin_tools.common import get_linked_devel_path
from catkin_tools.common import get_linked_devel_package_path
from catkin_tools.common import CATKIN_TOOLS_DIRNAME

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage
from catkin_tools.execution.stages import FunctionStage

from .commands.cmake import CMAKE_EXEC
from .commands.cmake import CMakeIOBufferProtocol
from .commands.make import MAKE_EXEC

from .job import create_build_space
from .job import create_env_file
from .job import get_env_file_path
from .job import get_package_build_space_path
from .job import makedirs

DEVEL_MANIFEST_FILENAME = 'devel_manifest.txt'
DEVEL_COLLISIONS_FILENAME = 'devel_collisions.txt'
DOT_CATKIN_FILENAME = '.catkin'

# List of files which shouldn't be copied
devel_product_blacklist = [
    DOT_CATKIN_FILENAME,
    os.path.join('etc', 'catkin', 'profile.d', '05.catkin_make.bash'),
    os.path.join('etc', 'catkin', 'profile.d', '05.catkin_make_isolated.bash'),
    os.path.join('etc', 'catkin', 'profile.d', '05.catkin-test-results.sh'),
    '.rosinstall',
    'env.sh',
    'setup.bash',
    'setup.zsh',
    'setup.sh',
    '_setup_util.py',
    'devel_manifest.txt']

# Synchronize access to the .catkin file
dot_catkin_file_lock = threading.Lock()

# Synchronize access to the devel collisions file
dest_collisions_file_lock = threading.Lock()


# Path helpers

def get_linked_devel_manifest_path(devel_space_abs, package_name):
    """The path to a given package's devel_manifest.txt"""
    return os.path.join(
        get_linked_devel_package_path(devel_space_abs, package_name),
        DEVEL_MANIFEST_FILENAME)


def get_linked_devel_collision_path(devel_space_abs):
    """Get the path to the devel collisoin file."""
    return os.path.join(
        get_linked_devel_path(devel_space_abs),
        DEVEL_COLLISIONS_FILENAME)


def get_bootstrap_path(devel_space_abs, mkdirs=False):
    """Get the path to the prebuild directory."""
    return os.path.join(
        get_linked_devel_path(devel_space_abs),
        'cstkin_tools_prebuild')


# .catkin file manipulation

def append_dot_catkin_file(logger, event_queue, devel_space_abs, package_source_abs):
    """
    Append the package source path to the .catkin file in the merged devel space

    This is normally done by catkin.
    """

    with dot_catkin_file_lock:
        if not os.path.exists(devel_space_abs):
            os.mkdir(devel_space_abs)
        dot_catkin_filename_abs = os.path.join(devel_space_abs, DOT_CATKIN_FILENAME)
        if os.path.exists(dot_catkin_filename_abs):
            with open(dot_catkin_filename_abs, 'r') as dot_catkin_file:
                dot_catkin_paths = dot_catkin_file.read().split(';')
            if package_source_abs not in dot_catkin_paths:
                with open(dot_catkin_filename_abs, 'a') as dot_catkin_file:
                    dot_catkin_file.write(';%s' % package_source_abs)
        else:
            with open(dot_catkin_filename_abs, 'w+') as dot_catkin_file:
                dot_catkin_file.write(package_source_abs)
    return 0


def clean_dot_catkin_file(logger, event_queue, devel_space_abs, package_name):
    """
    Remove a package source path from the .catkin file in the merged devel space
    """

    # Get the path to the package source directory
    linked_devel_path = get_linked_devel_package_path(devel_space_abs, package_name)
    devel_manifest_path = get_linked_devel_manifest_path(devel_space_abs, package_name)

    if not os.path.exists(linked_devel_path) or not os.path.exists(devel_manifest_path):
        return 0

    with open(devel_manifest_path, 'r') as devel_manifest:
        package_source_abs = devel_manifest.readline().strip()

    # Remove the package source directory from the .catkin file
    with dot_catkin_file_lock:
        dot_catkin_filename_abs = os.path.join(devel_space_abs, DOT_CATKIN_FILENAME)
        if os.path.exists(dot_catkin_filename_abs):
            dot_catkin_paths = []
            with open(dot_catkin_filename_abs, 'r') as dot_catkin_file:
                dot_catkin_paths = dot_catkin_file.read().split(';')
            if package_source_abs in dot_catkin_paths:
                dot_catkin_paths = [p for p in dot_catkin_paths if p != package_source_abs]
                with open(dot_catkin_filename_abs, 'w') as dot_catkin_file:
                    dot_catkin_file.write(';'.join(dot_catkin_paths))
    return 0


# Bootstrap files

SETUP_BOOTSTRAP_CMAKELISTS_TEMPLATE = """cmake_minimum_required(VERSION 2.8.7)
project(catkin_tools_prebuild)

find_package(catkin QUIET)

if(catkin_FOUND)
  catkin_package()
else()
  # Generate an error here which is more helpful than the normal one generated by CMake.
  # TODO: It's possible that we could just do this silently, instead.

  message(FATAL_ERROR
"The catkin CMake module was not found, but it is required to build a workspace.\
 To resolve this, please do one of the following, and try building again.

 1. Source the setup.sh file from an existing catkin workspace:
    source SETUP_FILE

 2. Extend another catkin workspace's result (install or devel) space:
    catkin config --extend RESULT_SPACE

 3. Set `catkin_DIR` to the directory containing `catkin-config.cmake`:
    catkin config --cmake-args -Dcatkin_DIR=CATKIN_CMAKE_CONFIG_PATH

 4. Add the catkin source package to your workspace's source space:
    cd SOURCE_SPACE && git clone https://github.com/ros/catkin.git")
endif()"""

SETUP_BOOTSTRAP_PACKAGE_XML_TEMPLATE = """<package>
  <name>catkin_tools_prebuild</name>
  <description>
    This package is used to generate catkin setup files.
  </description>
  <version>0.0.0</version>
  <license>BSD</license>
  <maintainer email="jbo@jhu.edu">jbohren</maintainer>
  <buildtool_depend>catkin</buildtool_depend>
</package>"""


def generate_setup_bootstrap(build_space_abs, devel_space_abs, force):
    """This generates a minimal Catkin package used to generate Catkin
    environment setup files in a merged devel space.

    :param build_space_abs: The path to a merged build space
    :param devel_space_abs: The path to a merged devel space
    :param force: Overwrite files if they exist
    :returns: 0 on success
    """

    # Get the path to the bootstrap package
    bootstrap_path = get_bootstrap_path(devel_space_abs, mkdirs=True)
    if not os.path.exists(bootstrap_path):
        mkdir_p(bootstrap_path)

    # Create CMakeLists.txt file
    cmakelists_txt_path = os.path.join(bootstrap_path, 'CMakeLists.txt')
    if force or not os.path.exists(cmakelists_txt_path):
        with open(cmakelists_txt_path, 'wb') as cmakelists_txt:
            cmakelists_txt.write(SETUP_BOOTSTRAP_CMAKELISTS_TEMPLATE.encode('utf-8'))

    # Create package.xml file
    package_xml_path = os.path.join(bootstrap_path, 'package.xml')
    if force or not os.path.exists(package_xml_path):
        with open(package_xml_path, 'wb') as package_xml:
            package_xml.write(SETUP_BOOTSTRAP_PACKAGE_XML_TEMPLATE.encode('utf-8'))

    # Create the build directory for this package
    mkdir_p(os.path.join(build_space_abs, 'catkin_tools_prebuild'))

    return 0


# symlink management


def clean_linked_files(logger, event_queue, devel_space_abs, files_that_collide, files_to_clean):
    """Removes a list of files and adjusts collison counts for colliding files.

    This function synchronizes access to the devel collisions file.

    :param devel_space_abs: absolute path to merged devel space
    :param files_that_collide: list of absolute paths to files that collide
    :param files_to_clean: list of absolute paths to files to clean
    """

    # Get paths
    devel_collisions_file_path = get_linked_devel_collision_path(devel_space_abs)

    with dest_collisions_file_lock:
        # Map from dest files to number of collisions
        dest_collisions = dict()

        # Load destination collisions file
        if os.path.exists(devel_collisions_file_path):
            with open(devel_collisions_file_path, 'r') as collisions_file:
                collisions_reader = csv.reader(collisions_file, delimiter=' ', quotechar='"')
                dest_collisions = dict([(path, int(count)) for path, count in collisions_reader])

        # Add collisions
        for dest_file in files_that_collide:
            if dest_file in dest_collisions:
                dest_collisions[dest_file] += 1
            else:
                dest_collisions[dest_file] = 1

        # Remove files that no longer collide
        for dest_file in files_to_clean:
            # Get the collisions
            n_collisions = dest_collisions.get(dest_file, 0)

            # Check collisions
            if n_collisions == 0:
                logger.out('Unlinking %s' % (dest_file))
                # Remove this link
                os.unlink(dest_file)
                # Remove any non-empty directories containing this file
                try:
                    os.removedirs(os.path.split(dest_file)[0])
                except OSError:
                    pass

            # Update collisions
            if n_collisions > 1:
                # Decrement the dest collisions dict
                dest_collisions[dest_file] -= 1
            elif n_collisions == 1:
                # Remove it from the dest collisions dict
                del dest_collisions[dest_file]

        # Load destination collisions file
        with open(devel_collisions_file_path, 'w') as collisions_file:
            collisions_writer = csv.writer(collisions_file, delimiter=' ', quotechar='"')
            for dest_file, count in dest_collisions.items():
                collisions_writer.writerow([dest_file, count])


def unlink_devel_products(logger, event_queue, devel_space_abs, package_name):
    """
    Remove all files listed in the devel manifest for the given package, as
    well as any empty directories containing those files.

    :param devel_space_abs: Path to a merged devel space.
    :param package_name: Name of the package whose files should be unlinked.
    """

    # Get paths
    linked_devel_path = get_linked_devel_package_path(devel_space_abs, package_name)
    devel_manifest_path = get_linked_devel_manifest_path(devel_space_abs, package_name)

    if not os.path.exists(linked_devel_path) or not os.path.exists(devel_manifest_path):
        return 0

    # List of files to clean
    files_to_clean = []

    # Read in devel_manifest.txt
    with open(devel_manifest_path, 'r') as devel_manifest:
        devel_manifest.readline()
        manifest_reader = csv.reader(devel_manifest, delimiter=' ', quotechar='"')

        # Remove all listed symlinks and empty directories
        for source_file, dest_file in manifest_reader:
            if not os.path.exists(dest_file):
                logger.err("Warning: Dest file doesn't exist, so it can't be removed: " + dest_file)
            elif not os.path.islink(dest_file):
                logger.err("Error: Dest file isn't a symbolic link: " + dest_file)
                return -1
            elif False and os.path.realpath(dest_file) != source_file:
                logger.err("Error: Dest file isn't a symbolic link to the expected file: " + dest_file)
                return -1
            else:
                # Clean the file or decrement the collision count
                files_to_clean.append(dest_file)

    # Remove all listed symlinks and empty directories which have been removed
    # after this build, and update the collision file
    clean_linked_files(logger, event_queue, devel_space_abs, [], files_to_clean)

    return 0


def link_devel_products(logger, event_queue, devel_space_abs, package_source_abs, package_name):
    """Link files from an isolated devel space into a merged one.

    This creates directories and symlinks in a merged devel space to a
    package's linked devel space.

    :param devel_space_abs: Path to a merged devel space
    :param package_source_abs: The path to the package source directory
    :param pacakge_name: The name of the package to link
    """

    # Get paths
    source_devel = get_linked_devel_package_path(devel_space_abs, package_name)
    devel_manifest_path = get_linked_devel_manifest_path(devel_space_abs, package_name)
    dest_devel = devel_space_abs

    # Pair of source/dest files or directories
    products = list()
    # List of files to clean
    files_to_clean = []
    # List of files that collide
    files_that_collide = []

    # Gather all of the files in the devel space
    for source_path, dirs, files in os.walk(source_devel):
        # compute destination path
        dest_path = os.path.join(dest_devel, os.path.relpath(source_path, source_devel))

        # create directories in the destination develspace
        for dirname in dirs:
            source_dir = os.path.join(source_path, dirname)
            dest_dir = os.path.join(dest_path, dirname)

            if not os.path.exists(dest_dir):
                # Create the dest directory if it doesn't exist
                os.mkdir(dest_dir)
            elif not os.path.isdir(dest_dir):
                logger.err('Error: Cannot create directory: ' + dest_dir)
                return -1

        # create symbolic links from the source to the dest
        for filename in files:

            # Don't link files on the blacklist
            if os.path.relpath(os.path.join(source_path, filename), source_devel) in devel_product_blacklist:
                continue

            source_file = os.path.join(source_path, filename)
            dest_file = os.path.join(dest_path, filename)

            # Store the source/dest pair
            products.append((source_file, dest_file))

            # Check if the symlink exists
            if os.path.exists(dest_file):
                if os.path.realpath(dest_file) != os.path.realpath(source_file):
                    # If the link links to a different file, report a warning and increment
                    # the collision counter for this path
                    logger.err('Warning: Cannot symlink from %s to existing file %s' % (source_file, dest_file))
                    # Increment link collision counter
                    files_that_collide.append(dest_file)
            else:
                # Create the symlink
                logger.out('Symlinking %s' % (dest_file))
                os.symlink(source_file, dest_file)

    # Load the old list of symlinked files for this package
    if os.path.exists(devel_manifest_path):
        with open(devel_manifest_path, 'r') as devel_manifest:
            manifest_reader = csv.reader(devel_manifest, delimiter=' ', quotechar='"')
            # Skip the package source directory
            devel_manifest.readline()
            # Read the previously-generated products
            for source_file, dest_file in manifest_reader:
                # print('Checking (%s, %s)' % (source_file, dest_file))
                if (source_file, dest_file) not in products:
                    # Clean the file or decrement the collision count
                    logger.out('Cleaning (%s, %s)' % (source_file, dest_file))
                    files_to_clean.append(dest_file)

    # Remove all listed symlinks and empty directories which have been removed
    # after this build, and update the collision file
    clean_linked_files(logger, event_queue, devel_space_abs, files_that_collide, files_to_clean)

    # Save the list of symlinked files
    with open(devel_manifest_path, 'w') as devel_manifest:
        # Write the path to the package source directory
        devel_manifest.write('%s\n' % package_source_abs)
        # Write all the products
        manifest_writer = csv.writer(devel_manifest, delimiter=' ', quotechar='"')
        for source_file, dest_file in products:
            manifest_writer.writerow([source_file, dest_file])

    return 0


# job factories

def create_catkin_build_job(context, package, package_path, dependencies, force_cmake, pre_clean):
    """Job class for building catkin packages"""

    # Package source space path
    pkg_dir = os.path.join(context.source_space_abs, package_path)

    # Package build space path
    build_space = get_package_build_space_path(context.build_space_abs, package.name)

    # Package devel space path
    if context.isolate_devel:
        devel_space = os.path.join(context.devel_space_abs, package.name)
    elif context.link_devel:
        devel_space = get_linked_devel_package_path(context.devel_space_abs, package.name)
    else:
        devel_space = context.devel_space_abs

    # Package install space path
    if context.isolate_install:
        install_space = os.path.join(context.install_space_abs, package.name)
    else:
        install_space = context.install_space_abs

    # Create job stages
    stages = []

    # Create package build space
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=build_space))

    # Create an environment file
    env_file_path = get_env_file_path(package, context)
    stages.append(FunctionStage(
        'envgen',
        create_env_file,
        package=package,
        context=context,
        env_file_path=env_file_path))

    # Only use it for building if the develspace is isolated
    env_prefix = []
    if context.isolate_devel:
        env_prefix = [env_file_path]

    # Construct CMake command
    makefile_path = os.path.join(build_space, 'Makefile')
    if not os.path.isfile(makefile_path) or force_cmake:
        stages.append(CommandStage(
            'cmake',
            ([env_file_path,
                CMAKE_EXEC,
                pkg_dir,
                '--no-warn-unused-cli',
                '-DCATKIN_DEVEL_PREFIX=' + devel_space,
                '-DCMAKE_INSTALL_PREFIX=' + install_space]
             + context.cmake_args),
            cwd=build_space,
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir),
            occupy_job=True
        ))
    else:
        stages.append(CommandStage(
            'check',
            env_prefix + [MAKE_EXEC, 'cmake_check_build_system'],
            cwd=build_space,
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir),
            occupy_job=True
        ))

    # Pre-clean command
    if pre_clean:
        make_args = handle_make_arguments(
            context.make_args + context.catkin_make_args)
        stages.append(CommandStage(
            'preclean',
            env_prefix + [MAKE_EXEC, 'clean'] + make_args,
            cwd=build_space,
        ))

    # Make command
    make_args = handle_make_arguments(
        context.make_args + context.catkin_make_args)
    stages.append(CommandStage(
        'make',
        env_prefix + [MAKE_EXEC] + make_args,
        cwd=build_space,
    ))

    # Symlink command if using a linked develspace
    if context.link_devel:
        stages.append(FunctionStage(
            'register',
            append_dot_catkin_file,
            devel_space_abs=context.devel_space_abs,
            package_source_abs=os.path.join(context.source_space_abs, package_path)
        ))
        stages.append(FunctionStage(
            'symlink',
            link_devel_products,
            devel_space_abs=context.devel_space_abs,
            package_source_abs=os.path.join(context.source_space_abs, package_path),
            package_name=package.name
        ))

    # Make install command, if installing
    if context.install:
        stages.append(CommandStage(
            'install',
            command=env_prefix + [MAKE_EXEC, 'install'],
            cwd=build_space))

    return Job(
        jid=package.name,
        deps=dependencies,
        stages=stages)


def create_catkin_tools_bootstrap_job(context, package, package_path, devel_space):
    """Job class for building catkin packages"""

    # Package source space path
    pkg_dir = os.path.join(context.source_space_abs, package_path)

    # Package build space path
    build_space = get_package_build_space_path(context.build_space_abs, package.name)

    # Package install space path
    if context.isolate_install:
        install_space = os.path.join(context.install_space_abs, package.name)
    else:
        install_space = context.install_space_abs

    # Create job stages
    stages = []

    # Create package build space
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=build_space))

    # Construct CMake command
    makefile_path = os.path.join(build_space, 'Makefile')
    stages.append(CommandStage(
        'cmake',
        [
            CMAKE_EXEC,
            pkg_dir,
            '--no-warn-unused-cli',
            '-DCATKIN_DEVEL_PREFIX=' + devel_space,
            '-DCMAKE_INSTALL_PREFIX=' + install_space
        ] + context.cmake_args,
        cwd=build_space,
        logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir)
    ))

    # Make command
    make_args = handle_make_arguments(
        context.make_args + context.catkin_make_args)
    stages.append(CommandStage(
        'make',
        [
            MAKE_EXEC
        ] + make_args,
        cwd=build_space,
    ))

    # Make install command, if installing
    if context.install:
        stages.append(CommandStage(
            'install',
            command=[MAKE_EXEC, 'install'],
            cwd=build_space))

    return Job(
        jid=package.name,
        deps=[],
        stages=stages)


def create_catkin_clean_job(context, package_name, dependencies):
    """Generate a Job that cleans a catkin package"""

    stages = []

    # Check if the build space exists
    build_space = get_package_build_space_path(context.build_space_abs, package_name)
    if not os.path.exists(build_space):
        # No-op
        return Job(jid=package_name, deps=dependencies, stages=[])

    # For isolated devel space, remove it entirely
    if context.isolate_devel:
        devel_space = os.path.join(context.devel_space_abs, package_name)
        return Job(
            jid=package_name,
            deps=dependencies,
            stages=[CommandStage(
                'clean',
                [CMAKE_EXEC, '-E', 'remove_directory', devel_space],
                cwd=build_space)])
    elif context.link_devel:
        devel_space = os.path.join(build_space, 'devel')
    else:
        devel_space = context.devel_space_abs

    # For isolated install space, remove it entirely
    if context.isolate_install:
        install_space = os.path.join(context.install_space_abs, package_name)
        return Job(
            jid=package_name,
            deps=dependencies,
            stages=[CommandStage(
                'clean',
                [CMAKE_EXEC, '-E', 'remove_directory', install_space],
                cwd=build_space)])
    else:
        install_space = context.install_space_abs

    # Clean symlinks if linked devel
    if context.link_devel:
        stages.append(FunctionStage(
            'unregister',
            clean_dot_catkin_file,
            devel_space_abs=context.devel_space_abs,
            package_name=package_name))
        stages.append(FunctionStage(
            'unlink',
            unlink_devel_products,
            devel_space_abs=context.devel_space_abs,
            package_name=package_name))
        stages.append(CommandStage(
            'rmdevel',
            [CMAKE_EXEC, '-E', 'remove_directory',
             get_linked_devel_package_path(context.devel_space_abs, package_name)],
            cwd=context.devel_space_abs))

    stages.append(CommandStage(
        'rmbuild',
        [CMAKE_EXEC, '-E', 'remove_directory', build_space],
        cwd=context.build_space_abs))

    return Job(
        jid=package_name,
        deps=dependencies,
        stages=stages)
