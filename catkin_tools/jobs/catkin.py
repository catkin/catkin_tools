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

from catkin_tools.runner import run_command

from catkin_tools.utils import which

from .commands.cmake import CMakeCommand
from .commands.cmake import CMAKE_EXEC
from .commands.make import MakeCommand
from .commands.make import MAKE_EXEC
from .commands.python_command import PythonCommand

from .job import create_build_space
from .job import create_env_file
from .job import BuildJob
from .job import CleanJob

CATKIN_TOOLS_DIRNAME = '.catkin_tools'
DEVEL_MANIFEST_FILENAME = 'devel_manifest.txt'
DEVEL_COLLISIONS_FILENAME = 'devel_collisions.txt'
DOT_CATKIN_FILENAME = '.catkin'
LINKED_DEVEL_DIRNAME = 'linked_devel'

# List of files which shouldn't be copied
devel_product_blacklist = [
    DOT_CATKIN_FILENAME,
    '.rosinstall',
    'env.sh',
    'setup.bash',
    'setup.zsh',
    'setup.sh',
    '_setup_util.py']

# Synchronize access to the .catkin file
dot_catkin_file_lock = threading.Lock()

# Synchronize access to the devel collisions file
dest_collisions_file_lock = threading.Lock()


def mkdir_p(path):
    """Equivalent to UNIX mkdir -p"""
    if os.path.exists(path):
        return
    try:
        return os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


# Path helpers

def get_devel_manifest_path(devel_space_abs, package_name, mkdirs=False):
    """Get the path to the devel manifest for this package."""

    devel_manifest_dir = os.path.join(
        devel_space_abs,
        CATKIN_TOOLS_DIRNAME,
        package_name)

    if mkdirs:
        mkdir_p(devel_manifest_dir)

    return os.path.join(devel_manifest_dir, DEVEL_MANIFEST_FILENAME)


def get_linked_devel_path(devel_space_abs, package_name, mkdirs=False):
    """Get the path to the linked devel space for this package."""

    linked_devel_space_dir = os.path.join(
        devel_space_abs,
        CATKIN_TOOLS_DIRNAME,
        package_name,
        LINKED_DEVEL_DIRNAME)

    if mkdirs:
        mkdir_p(linked_devel_space_dir)

    return linked_devel_space_dir


def get_devel_collision_path(devel_space_abs, mkdirs=False):
    """Get the path to the devel collisoin file."""

    devel_catkin_tools_dir = os.path.join(
        devel_space_abs,
        CATKIN_TOOLS_DIRNAME)

    if mkdirs:
        mkdir_p(devel_catkin_tools_dir)

    return os.path.join(devel_catkin_tools_dir, DEVEL_COLLISIONS_FILENAME)


def get_bootstrap_path(devel_space_abs, mkdirs=False):
    """Get the path to the bootstrap directory."""

    bootstrap_dir = os.path.join(
        devel_space_abs,
        CATKIN_TOOLS_DIRNAME,
        'bootstrap')

    if mkdirs:
        mkdir_p(bootstrap_dir)

    return bootstrap_dir


# .catkin file manipulation

def append_dot_catkin_file(devel_space_abs, package_source_abs):
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
                with open(dot_catkin_filename_abs, 'ab') as dot_catkin_file:
                    dot_catkin_file.write(';%s' % package_source_abs)
        else:
            with open(dot_catkin_filename_abs, 'w+') as dot_catkin_file:
                dot_catkin_file.write(package_source_abs)
    return 0


def clean_dot_catkin_file(devel_space_abs, package_name):
    """
    Remove a package source path from the .catkin file in the merged devel space
    """

    # Get the path to the package source directory
    linked_devel_path = get_linked_devel_path(devel_space_abs, package_name, mkdirs=False)
    devel_manifest_path = get_devel_manifest_path(devel_space_abs, package_name, mkdirs=False)

    if not os.path.exists(linked_devel_path) or not os.path.exists(devel_manifest_path):
        return 0

    with open(devel_manifest_path, 'rb') as devel_manifest:
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
                with open(dot_catkin_filename_abs, 'wb') as dot_catkin_file:
                    dot_catkin_file.write(';'.join(dot_catkin_paths))
    return 0


# Bootstrap files

SETUP_BOOTSTRAP_CMAKELISTS_TEMPLATE = """cmake_minimum_required(VERSION 2.8.3)
project(catkin_tools_bootstrap)
find_package(catkin REQUIRED)
catkin_package()"""

SETUP_BOOTSTRAP_PACKAGE_XML_TEMPLATE = """<package>
  <name>catkin_tools_bootstrap</name>
  <description>
    This package is used to generate catkin setup files.
  </description>
  <version>0.0.0</version>
  <license>BSD</license>
  <maintainer email="jbo@jhu.edu">jbohren</maintainer>
  <buildtool_depend>catkin</buildtool_depend>
</package>"""


def generate_setup_bootstrap(build_space_abs, devel_space_abs):
    """This generates a minimal Catkin package used to generate Catkin
    environment setup files in a merged devel space.

    :param build_space_abs: The path to a merged build space
    :param devel_space_abs: The path to a merged devel space
    :returns: 0 on success
    """

    bootstrap_path = get_bootstrap_path(devel_space_abs, mkdirs=True)

    # Create CMakeLists.txt file
    cmakelists_txt_path = os.path.join(bootstrap_path, 'CMakeLists.txt')
    if not os.path.exists(cmakelists_txt_path):
        with open(cmakelists_txt_path, 'wb') as cmakelists_txt:
            cmakelists_txt.write(SETUP_BOOTSTRAP_CMAKELISTS_TEMPLATE)

    # Create package.xml file
    package_xml_path = os.path.join(bootstrap_path, 'package.xml')
    if not os.path.exists(package_xml_path):
        with open(package_xml_path, 'wb') as package_xml:
            package_xml.write(SETUP_BOOTSTRAP_PACKAGE_XML_TEMPLATE)

    # Create the build directory for this package
    mkdir_p(os.path.join(build_space_abs, 'catkin_tools_bootstrap'))

    return 0


# symlink management


def clean_linked_files(devel_space_abs, files_that_collide, files_to_clean):
    """Removes a list of files and adjusts collison counts for colliding files.

    This function synchronizes access to the devel collisions file.

    :param devel_space_abs: absolute path to merged devel space
    :param files_that_collide: list of absolute paths to files that collide
    :param files_to_clean: list of absolute paths to files to clean
    """

    # Get paths
    devel_collisions_file_path = get_devel_collision_path(devel_space_abs, mkdirs=True)

    with dest_collisions_file_lock:
        # Map from dest files to number of collisions
        dest_collisions = dict()

        # Load destination collisions file
        if os.path.exists(devel_collisions_file_path):
            with open(devel_collisions_file_path, 'rb') as collisions_file:
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
                print('Unlinking %s' % (dest_file))
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
        with open(devel_collisions_file_path, 'wb') as collisions_file:
            collisions_writer = csv.writer(collisions_file, delimiter=' ', quotechar='"')
            for dest_file, count in dest_collisions.items():
                collisions_writer.writerow([dest_file, count])


def unlink_devel_products(devel_space_abs, package_name):
    """
    Remove all files listed in the devel manifest for the given package, as
    well as any empty directories containing those files.

    :param devel_space_abs: Path to a merged devel space.
    :param package_name: Name of the package whose files should be unlinked.
    """

    # Get paths
    linked_devel_path = get_linked_devel_path(devel_space_abs, package_name, mkdirs=False)
    devel_manifest_path = get_devel_manifest_path(devel_space_abs, package_name, mkdirs=False)

    if not os.path.exists(linked_devel_path) or not os.path.exists(devel_manifest_path):
        return 0

    # List of files to clean
    files_to_clean = []

    # Read in devel_manifest.txt
    with open(devel_manifest_path, 'rb') as devel_manifest:
        devel_manifest.readline()
        manifest_reader = csv.reader(devel_manifest, delimiter=' ', quotechar='"')

        # Remove all listed symlinks and empty directories
        for source_file, dest_file in manifest_reader:
            if not os.path.exists(dest_file):
                print("WARNING: Dest file doesn't exist, so it can't be removed: " + dest_file)
            elif not os.path.islink(dest_file):
                print("ERROR: Dest file isn't a symbolic link: " + dest_file)
                return -1
            elif False and os.path.realpath(dest_file) != source_file:
                print("ERROR: Dest file isn't a symbolic link to the expected file: " + dest_file)
                return -1
            else:
                # Clean the file or decrement the collision count
                files_to_clean.append(dest_file)

    # Remove all listed symlinks and empty directories which have been removed
    # after this build, and update the collision file
    clean_linked_files(devel_space_abs, [], files_to_clean)

    return 0


def link_devel_products(devel_space_abs, package_source_abs, package_name):
    """Link files from an isolated devel space into a merged one.

    This creates directories and symlinks in a merged devel space to a
    package's linked devel space.

    :param devel_space_abs: Path to a merged devel space
    :param package_source_abs: The path to the package source directory
    :param pacakge_name: The name of the package to link
    """

    # Get paths
    source_devel = get_linked_devel_path(devel_space_abs, package_name, mkdirs=True)
    devel_manifest_path = get_devel_manifest_path(devel_space_abs, package_name, mkdirs=True)
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
                print('ERROR: cannot create directory: ' + dest_dir)
                return -1

        # create symbolic links from the source to the dest
        for filename in files:

            if source_path == source_devel and filename in devel_product_blacklist:
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
                    print('WARNING: Cannot symlink from %s to existing file %s' % (source_file, dest_file))
                    # Increment link collision counter
                    files_that_collide.append(dest_file)
            else:
                # Create the symlink
                print('Symlinking %s' % (dest_file))
                os.symlink(source_file, dest_file)

    # Load the old list of symlinked files for this package
    if os.path.exists(devel_manifest_path):
        with open(devel_manifest_path, 'rb') as devel_manifest:
            manifest_reader = csv.reader(devel_manifest, delimiter=' ', quotechar='"')
            # Skip the package source directory
            devel_manifest.readline()
            # Read the previously-generated products
            for source_file, dest_file in manifest_reader:
                # print('Checking (%s, %s)' % (source_file, dest_file))
                if (source_file, dest_file) not in products:
                    # Clean the file or decrement the collision count
                    print('Cleaning (%s, %s)' % (source_file, dest_file))
                    files_to_clean.append(dest_file)

    # Remove all listed symlinks and empty directories which have been removed
    # after this build, and update the collision file
    clean_linked_files(devel_space_abs, files_that_collide, files_to_clean)

    # Save the list of symlinked files
    with open(devel_manifest_path, 'wb') as devel_manifest:
        # Write the path to the package source directory
        devel_manifest.write('%s\n' % package_source_abs)
        # Write all the products
        manifest_writer = csv.writer(devel_manifest, delimiter=' ', quotechar='"')
        for source_file, dest_file in products:
            manifest_writer.writerow([source_file, dest_file])

    return 0


# jobs

class CatkinBuildJob(BuildJob):

    """Job class for building catkin packages"""

    def __init__(self, context, package, package_path, force_cmake):
        super(CatkinBuildJob, self).__init__(context, package, package_path, force_cmake)
        self.commands = self.get_commands()

    def get_commands(self):
        commands = []
        # Setup build variables
        pkg_dir = os.path.join(self.context.source_space_abs, self.package_path)
        build_space = create_build_space(self.context.build_space_abs, self.package.name)
        # Devel space path
        if self.context.isolate_devel:
            devel_space = os.path.join(self.context.devel_space_abs, self.package.name)
        elif self.context.link_devel:
            devel_space = get_linked_devel_path(self.context.devel_space_abs, self.package.name, mkdirs=True)
        else:
            devel_space = self.context.devel_space_abs
        # Install space path
        if self.context.isolate_install:
            install_space = os.path.join(self.context.install_space_abs, self.package.name)
        else:
            install_space = self.context.install_space_abs
        # Create an environment file
        env_cmd = create_env_file(self.package, self.context)
        # CMake command
        makefile_path = os.path.join(build_space, 'Makefile')
        if not os.path.isfile(makefile_path) or self.force_cmake:
            commands.append(CMakeCommand(
                env_cmd,
                [
                    CMAKE_EXEC,
                    pkg_dir,
                    '-DCATKIN_DEVEL_PREFIX=' + devel_space,
                    '-DCMAKE_INSTALL_PREFIX=' + install_space
                ] + self.context.cmake_args,
                build_space
            ))
        else:
            commands.append(MakeCommand(env_cmd, [MAKE_EXEC, 'cmake_check_build_system'], build_space))
        # Make command
        commands.append(MakeCommand(
            env_cmd,
            [MAKE_EXEC] +
            handle_make_arguments(self.context.make_args + self.context.catkin_make_args),
            build_space
        ))

        # Symlink command if linked devel
        if self.context.link_devel:
            commands.extend([
                PythonCommand(
                    append_dot_catkin_file,
                    {'devel_space_abs': self.context.devel_space_abs,
                     'package_source_abs': os.path.join(self.context.source_space_abs, self.package_path)},
                    self.context.devel_space_abs),
                PythonCommand(
                    link_devel_products,
                    {'devel_space_abs': self.context.devel_space_abs,
                     'package_source_abs': os.path.join(self.context.source_space_abs, self.package_path),
                     'package_name': self.package.name},
                    self.context.devel_space_abs),
            ])

        # Make install command, if installing
        if self.context.install:
            commands.append(InstallCommand(env_cmd, [MAKE_EXEC, 'install'], build_space))
        return commands


class CatkinCleanJob(CleanJob):

    """Job class for cleaning catkin packages"""

    def __init__(self, context, package_name):
        super(CatkinCleanJob, self).__init__(context, package_name)
        self.commands = self.get_commands()

    def get_commands(self):
        commands = []

        # Check if the build space exists
        build_space = os.path.join(self.context.build_space_abs, self.package_name)
        if not os.path.exists(build_space):
            return commands

        # For isolated devel space, remove it entirely
        if self.context.isolate_devel:
            devel_space = os.path.join(self.context.devel_space_abs, self.package_name)
            commands.append(CMakeCommand(None, [CMAKE_EXEC, '-E', 'remove_directory', devel_space], build_space))
            return commands
        elif self.context.link_devel:
            devel_space = os.path.join(build_space, 'devel')
        else:
            devel_space = self.context.devel_space_abs

        # For isolated install space, remove it entirely
        if self.context.isolate_install:
            install_space = os.path.join(self.context.install_space_abs, self.package_name)
            commands.append(CMakeCommand(None, [CMAKE_EXEC, '-E', 'remove_directory', install_space], build_space))
            return commands
        else:
            install_space = self.context.install_space_abs

        # Symlink command if linked devel
        if self.context.link_devel:
            commands.extend([
                PythonCommand(
                    clean_dot_catkin_file,
                    {'devel_space_abs': self.context.devel_space_abs,
                     'package_name': self.package_name},
                    self.context.devel_space_abs),
                PythonCommand(
                    unlink_devel_products,
                    {'devel_space_abs': self.context.devel_space_abs,
                     'package_name': self.package_name},
                    self.context.devel_space_abs),
            ])

        return commands
