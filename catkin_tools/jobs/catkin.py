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
import os

try:
    from md5 import md5
except ImportError:
    from hashlib import md5

from catkin_tools.argument_parsing import handle_make_arguments

from catkin_tools.common import mkdir_p

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage
from catkin_tools.execution.stages import FunctionStage

from .commands.cmake import CMAKE_EXEC
from .commands.cmake import CMakeIOBufferProtocol
from .commands.cmake import CMakeMakeIOBufferProtocol
from .commands.cmake import get_installed_files
from .commands.make import MAKE_EXEC

from .utils import copyfiles
from .utils import get_env_loader
from .utils import makedirs
from .utils import rmfiles


def get_prebuild_package(build_space_abs, devel_space_abs, force):
    """This generates a minimal Catkin package used to generate Catkin
    environment setup files in a merged devel space.

    :param build_space_abs: The path to a merged build space
    :param devel_space_abs: The path to a merged devel space
    :param force: Overwrite files if they exist
    :returns: source directory path
    """

    # Get the path to the prebuild package
    prebuild_path = os.path.join(build_space_abs, 'catkin_tools_prebuild')
    if not os.path.exists(prebuild_path):
        mkdir_p(prebuild_path)

    # Create CMakeLists.txt file
    cmakelists_txt_path = os.path.join(prebuild_path, 'CMakeLists.txt')
    if force or not os.path.exists(cmakelists_txt_path):
        with open(cmakelists_txt_path, 'wb') as cmakelists_txt:
            cmakelists_txt.write(SETUP_PREBUILD_CMAKELISTS_TEMPLATE.encode('utf-8'))

    # Create package.xml file
    package_xml_path = os.path.join(prebuild_path, 'package.xml')
    if force or not os.path.exists(package_xml_path):
        with open(package_xml_path, 'wb') as package_xml:
            package_xml.write(SETUP_PREBUILD_PACKAGE_XML_TEMPLATE.encode('utf-8'))

    # Create the build directory for this package
    mkdir_p(os.path.join(build_space_abs, 'catkin_tools_prebuild'))

    return prebuild_path


def clean_linked_files(
        logger,
        event_queue,
        metadata_path,
        files_that_collide,
        files_to_clean,
        dry_run):
    """Removes a list of files and adjusts collison counts for colliding files.

    This function synchronizes access to the devel collisions file.

    :param devel_space_abs: absolute path to merged devel space
    :param files_that_collide: list of absolute paths to files that collide
    :param files_to_clean: list of absolute paths to files to clean
    """

    # Get paths
    devel_collisions_file_path = os.path.join(metadata_path, 'devel_collisions.txt')

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
            logger.out('Unlinking: {}'.format(dest_file))
            # Remove this link
            if not dry_run:
                if os.path.exists(dest_file):
                    try:
                        os.unlink(dest_file)
                    except OSError:
                        logger.err('Could not unlink: {}'.format(dest_file))
                        raise
                    # Remove any non-empty directories containing this file
                    try:
                        os.removedirs(os.path.split(dest_file)[0])
                    except OSError:
                        pass
                else:
                    logger.out('Already unlinked: {}')

        # Update collisions
        if n_collisions > 1:
            # Decrement the dest collisions dict
            dest_collisions[dest_file] -= 1
        elif n_collisions == 1:
            # Remove it from the dest collisions dict
            del dest_collisions[dest_file]

    # Load destination collisions file
    if not dry_run:
        with open(devel_collisions_file_path, 'w') as collisions_file:
            collisions_writer = csv.writer(collisions_file, delimiter=' ', quotechar='"')
            for dest_file, count in dest_collisions.items():
                collisions_writer.writerow([dest_file, count])


def unlink_devel_products(
        logger,
        event_queue,
        devel_space_abs,
        private_devel_path,
        metadata_path,
        package_metadata_path,
        dry_run):
    """
    Remove all files listed in the devel manifest for the given package, as
    well as any empty directories containing those files.

    :param devel_space_abs: Path to a merged devel space.
    :param private_devel_path: Path to the private devel space
    :param devel_manifest_path: Path to the directory containing the package's
    catkin_tools metadata
    """

    # Check paths
    if not os.path.exists(private_devel_path):
        logger.err('Warning: No private devel path found at `{}`'.format(private_devel_path))
        return 0

    devel_manifest_file_path = os.path.join(package_metadata_path, DEVEL_MANIFEST_FILENAME)
    if not os.path.exists(devel_manifest_file_path):
        logger.err('Error: No devel manifest found at `{}`'.format(devel_manifest_file_path))
        return 1

    # List of files to clean
    files_to_clean = []

    # Read in devel_manifest.txt
    with open(devel_manifest_file_path, 'r') as devel_manifest:
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

    # Remove all listed symli and empty directories which have been removed
    # after this build, and update the collision file
    clean_linked_files(logger, event_queue, metadata_path, [], files_to_clean, dry_run)

    return 0


def link_devel_products(
        logger, event_queue,
        package,
        package_path,
        devel_manifest_path,
        source_devel_path,
        dest_devel_path,
        metadata_path,
        prebuild):
    """Link files from an isolated devel space into a merged one.

    This creates directories and symlinks in a merged devel space to a
    package's linked devel space.
    """

    # Create the devel manifest path if necessary
    mkdir_p(devel_manifest_path)

    # Construct manifest file path
    devel_manifest_file_path = os.path.join(devel_manifest_path, DEVEL_MANIFEST_FILENAME)

    # Pair of source/dest files or directories
    products = list()
    # List of files to clean
    files_to_clean = []
    # List of files that collide
    files_that_collide = []

    # Select the blacklist
    blacklist = DEVEL_LINK_PREBUILD_BLACKLIST if prebuild else DEVEL_LINK_BLACKLIST

    # Gather all of the files in the devel space
    for source_path, dirs, files in os.walk(source_devel_path):
        # compute destination path
        dest_path = os.path.join(dest_devel_path, os.path.relpath(source_path, source_devel_path))

        # create directories in the destination develspace
        for dirname in dirs:
            dest_dir = os.path.join(dest_path, dirname)

            if not os.path.exists(dest_dir):
                # Create the dest directory if it doesn't exist
                os.mkdir(dest_dir)
            elif not os.path.isdir(dest_dir):
                logger.err('Error: Cannot create directory: ' + dest_dir)
                return -1

        # create symbolic links from the source to the dest
        for filename in files:

            # Don't link files on the blacklist unless this is a prebuild package
            if os.path.relpath(os.path.join(source_path, filename), source_devel_path) in blacklist:
                continue

            source_file = os.path.join(source_path, filename)
            dest_file = os.path.join(dest_path, filename)

            # Store the source/dest pair
            products.append((source_file, dest_file))

            # Check if the symlink exists
            if os.path.exists(dest_file):
                if os.path.realpath(dest_file) != os.path.realpath(source_file):
                    # Compute hashes for colliding files
                    source_hash = md5(open(os.path.realpath(source_file)).read().encode('utf-8')).hexdigest()
                    dest_hash = md5(open(os.path.realpath(dest_file)).read().encode('utf-8')).hexdigest()
                    # If the link links to a different file, report a warning and increment
                    # the collision counter for this path
                    if dest_hash != source_hash:
                        logger.err('Warning: Cannot symlink from %s to existing file %s' % (source_file, dest_file))
                        logger.err('Warning: Source hash: {}'.format(source_hash))
                        logger.err('Warning: Dest hash: {}'.format(dest_hash))
                    # Increment link collision counter
                    files_that_collide.append(dest_file)
                else:
                    logger.out('Linked: ({}, {})'.format(source_file, dest_file))
            else:
                # Create the symlink
                logger.out('Symlinking %s' % (dest_file))
                try:
                    os.symlink(source_file, dest_file)
                except OSError:
                    logger.err('Could not create symlink `{}` referencing `{}`'.format(dest_file, source_file))
                    raise

    # Load the old list of symlinked files for this package
    if os.path.exists(devel_manifest_file_path):
        with open(devel_manifest_file_path, 'r') as devel_manifest:
            manifest_reader = csv.reader(devel_manifest, delimiter=' ', quotechar='"')
            # Skip the package source directory
            devel_manifest.readline()
            # Read the previously-generated products
            for source_file, dest_file in manifest_reader:
                # print('Checking (%s, %s)' % (source_file, dest_file))
                if (source_file, dest_file) not in products:
                    # Clean the file or decrement the collision count
                    logger.out('Cleaning: (%s, %s)' % (source_file, dest_file))
                    files_to_clean.append(dest_file)

    # Remove all listed symlinks and empty directories which have been removed
    # after this build, and update the collision file
    try:
        clean_linked_files(logger, event_queue, metadata_path, files_that_collide, files_to_clean, dry_run=False)
    except:
        logger.err('Could not clean linked files.')
        raise

    # Save the list of symlinked files
    with open(devel_manifest_file_path, 'w') as devel_manifest:
        # Write the path to the package source directory
        devel_manifest.write('%s\n' % package_path)
        # Write all the products
        manifest_writer = csv.writer(devel_manifest, delimiter=' ', quotechar='"')
        for source_file, dest_file in products:
            manifest_writer.writerow([source_file, dest_file])

    return 0


def ctr_nuke(logger, event_queue, prefix):
    """Adds an env-hook which clears the catkin and ros test results dir."""

    ctr_nuke_path = os.path.join(prefix, 'etc', 'catkin', 'profile.d')
    ctr_nuke_filename = os.path.join(ctr_nuke_path, '06-ctr-nuke.sh')
    mkdir_p(ctr_nuke_path)
    if not os.path.exists(ctr_nuke_filename):
        with open(ctr_nuke_filename, 'w') as ctr_nuke_file:
            ctr_nuke_file.write(CTR_NUKE_SH)
    return 0


def create_catkin_build_job(context, package, package_path, dependencies, force_cmake, pre_clean, prebuild=False):
    """Job class for building catkin packages"""

    # Package source space path
    pkg_dir = os.path.join(context.source_space_abs, package_path)

    # Package build space path
    build_space = context.package_build_space(package)
    # Package devel space path
    devel_space = context.package_devel_space(package)
    # Package install space path
    install_space = context.package_install_space(package)
    # Package metadata path
    metadata_path = context.package_metadata_path(package)

    # Create job stages
    stages = []

    # Create package build space
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=build_space
    ))

    # Create package metadata dir
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=metadata_path
    ))

    # Copy source manifest
    stages.append(FunctionStage(
        'cache-manifest',
        copyfiles,
        source_paths=[os.path.join(context.source_space_abs, package_path, 'package.xml')],
        dest_path=os.path.join(metadata_path, 'package.xml')
    ))

    # Define test results directory
    catkin_test_results_dir = os.path.join(build_space, 'test_results')
    # Always override the CATKIN and ROS _TEST_RESULTS_DIR environment variables.
    # This is in order to avoid cross talk due to parallel builds.
    # This is only needed for ROS Hydro and earlier (the problem was addressed upstream in Indigo).
    # See: https://github.com/catkin/catkin_tools/issues/139
    ctr_env = {
        'CATKIN_TEST_RESULTS_DIR': catkin_test_results_dir,
        'ROS_TEST_RESULTS_DIR': catkin_test_results_dir
    }

    # Only run CMake if the Makefile doesn't exist or if --force-cmake is given
    # TODO: This would need to be different with `cmake --build`
    makefile_path = os.path.join(build_space, 'Makefile')

    if not os.path.isfile(makefile_path) or force_cmake:

        # Create an env-hook which clears the catkin and ros test results environment variable.
        stages.append(FunctionStage(
            'ctr-nuke',
            ctr_nuke,
            prefix=context.package_dest_path(package)
        ))

        # CMake command
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
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir),
            occupy_job=True
        ))
    else:
        # Check buildsystem command
        stages.append(CommandStage(
            'check',
            [MAKE_EXEC, 'cmake_check_build_system'],
            cwd=build_space,
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir),
            occupy_job=True
        ))

    # Filter make arguments
    make_args = handle_make_arguments(
        context.make_args +
        context.catkin_make_args)

    # Determine if the catkin test results env needs to be overridden
    env_overrides = ctr_env if 'test' in make_args else {}

    # Pre-clean command
    if pre_clean:
        # TODO: Remove target args from `make_args`
        stages.append(CommandStage(
            'preclean',
            [MAKE_EXEC, 'clean'] + make_args,
            cwd=build_space,
        ))

    # Make command
    stages.append(CommandStage(
        'make',
        [MAKE_EXEC] + make_args,
        cwd=build_space,
        env_overrides=env_overrides,
        logger_factory=CMakeMakeIOBufferProtocol.factory
    ))

    # Symlink command if using a linked develspace
    if context.link_devel:
        stages.append(FunctionStage(
            'symlink',
            link_devel_products,
            locked_resource='symlink-collisions-file',
            package=package,
            package_path=package_path,
            devel_manifest_path=context.package_metadata_path(package),
            source_devel_path=context.package_devel_space(package),
            dest_devel_path=context.devel_space_abs,
            metadata_path=context.metadata_path(),
            prebuild=prebuild
        ))

    # Make install command, if installing
    if context.install:
        stages.append(CommandStage(
            'install',
            [MAKE_EXEC, 'install'],
            cwd=build_space,
            logger_factory=CMakeMakeIOBufferProtocol.factory,
            locked_resource='installspace'
        ))

    return Job(
        jid=package.name,
        deps=dependencies,
        env_loader=get_env_loader(package, context),
        stages=stages)


def create_catkin_clean_job(
        context,
        package,
        package_path,
        dependencies,
        dry_run,
        clean_build,
        clean_devel,
        clean_install):
    """Generate a Job that cleans a catkin package"""

    stages = []

    # Package build space path
    build_space = context.package_build_space(package)
    # Package metadata path
    metadata_path = context.package_metadata_path(package)

    # Remove installed files
    if clean_install:
        installed_files = get_installed_files(context.package_metadata_path(package))
        stages.append(FunctionStage(
            'cleaninstall',
            rmfiles,
            paths=sorted(installed_files),
            remove_empty=True,
            empty_root=context.install_space_abs,
            dry_run=dry_run))

    # Remove products in develspace
    if clean_devel:
        if context.merge_devel:
            # Remove build targets from devel space
            stages.append(CommandStage(
                'clean',
                [MAKE_EXEC, 'clean'],
                cwd=build_space,
            ))
        elif context.link_devel:
            # Remove symlinked products
            stages.append(FunctionStage(
                'unlink',
                unlink_devel_products,
                locked_resource='symlink-collisions-file',
                devel_space_abs=context.devel_space_abs,
                private_devel_path=context.package_private_devel_path(package),
                metadata_path=context.metadata_path(),
                package_metadata_path=context.package_metadata_path(package),
                dry_run=dry_run
            ))

            # Remove devel space
            stages.append(FunctionStage(
                'rmdevel',
                rmfiles,
                paths=[context.package_private_devel_path(package)],
                dry_run=dry_run))
        elif context.isolate_devel:
            # Remove devel space
            stages.append(FunctionStage(
                'rmdevel',
                rmfiles,
                paths=[context.package_devel_space(package)],
                dry_run=dry_run))

    # Remove build space
    if clean_build:
        stages.append(FunctionStage(
            'rmbuild',
            rmfiles,
            paths=[build_space],
            dry_run=dry_run))

    # Remove cached metadata
    if clean_build and clean_devel and clean_install:
        stages.append(FunctionStage(
            'rmmetadata',
            rmfiles,
            paths=[metadata_path],
            dry_run=dry_run))

    return Job(
        jid=package.name,
        deps=dependencies,
        env_loader=get_env_loader(package, context),
        stages=stages)


description = dict(
    build_type='catkin',
    description="Builds a catkin package.",
    create_build_job=create_catkin_build_job,
    create_clean_job=create_catkin_clean_job
)


CTR_NUKE_SH = """\
#!/usr/bin/env sh
unset CATKIN_TEST_RESULTS_DIR
unset ROS_TEST_RESULTS_DIR
"""

DEVEL_MANIFEST_FILENAME = 'devel_manifest.txt'

# List of files which shouldn't be copied
DEVEL_LINK_PREBUILD_BLACKLIST = [
    '.catkin',
    '.rosinstall',
]
DEVEL_LINK_BLACKLIST = DEVEL_LINK_PREBUILD_BLACKLIST + [
    os.path.join('etc', 'catkin', 'profile.d', '05.catkin_make.bash'),
    os.path.join('etc', 'catkin', 'profile.d', '05.catkin_make_isolated.bash'),
    os.path.join('etc', 'catkin', 'profile.d', '05.catkin-test-results.sh'),
    'env.sh',
    'setup.bash',
    'setup.zsh',
    'setup.sh',
    '_setup_util.py',
]

# CMakeLists.txt for prebuild package
SETUP_PREBUILD_CMAKELISTS_TEMPLATE = """\
cmake_minimum_required(VERSION 2.8.7)
project(catkin_tools_prebuild)

find_package(catkin QUIET)

if(catkin_FOUND)
  catkin_package()
else()
  # Generate an error here which is more helpful than the normal one generated by CMake.
  # TODO: It's possible that we could just do this silently, instead.

  message(FATAL_ERROR
"The catkin CMake module was not found, but it is required to build a linked workspace.\
 To resolve this, please do one of the following, and try building again.

 1. Source the setup.sh file from an existing catkin workspace:
    source SETUP_FILE

 2. Extend another catkin workspace's result (install or devel) space:
    catkin config --extend RESULT_SPACE

 3. Set `catkin_DIR` to the directory containing `catkin-config.cmake`:
    catkin config --cmake-args -Dcatkin_DIR=CATKIN_CMAKE_CONFIG_PATH

 4. Add the catkin source package to your workspace's source space:
    cd SOURCE_SPACE && git clone https://github.com/ros/catkin.git")
endif()
"""

# package.xml file for prebuild package
SETUP_PREBUILD_PACKAGE_XML_TEMPLATE = """\
<package>
  <name>catkin_tools_prebuild</name>
  <description>
    This package is used to generate catkin setup files.
  </description>
  <version>0.0.0</version>
  <license>BSD</license>
  <maintainer email="jbo@jhu.edu">jbohren</maintainer>
  <buildtool_depend>catkin</buildtool_depend>
</package>
"""
