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

from __future__ import print_function

import os
import shutil

from catkin_pkg.packages import find_packages
from catkin_tools import metadata

# Exempt build directories
# See https://github.com/catkin/catkin_tools/issues/82
exempt_build_dirs = ['build_logs']


def prepare_arguments(parser):
    metadata_group = parser.add_mutually_exclusive_group()
    only_group = parser.add_mutually_exclusive_group()

    add = parser.add_argument
    madd = metadata_group.add_argument
    oadd = only_group.add_argument

    # Non-mutually-exclusive args
    add('workspace', nargs='?', default=os.getcwd(),
        help='The path to the catkin workspace to clean. Default: the workspace'
        'containing current working directory')

    # Metadata group
    madd('-m', '--metadata', action='store_true', default=False,
         help='Delete the metadata for the given workspace')
    madd('--metadata-dir', action='store_true', default=False,
         help='Delete the metadata for the given workspace, including the .catkin_tools marker directory')

    # Only group
    oadd('-c', '--ccache-only', action='store_true', default=False,
         help='Only clear the CMakeCache for each package, but leave build and devel spaces.')
    oadd('-o', '--orphans-only', action='store_true', default=False,
         help='Remove the develspace but only remove build directories whose '
         'source packages are no longer enabled or in the source space (this '
         'might require --force-cmake on the next build)')
    oadd('-d', '--devel-only', action='store_true', default=False,
         help='Only remove the develspace (this might require --force-cmake '
         'on the next build)')

    return parser


def main(opts):
    # Get the workspace
    marked_workspace = metadata.find_enclosing_workspace(opts.workspace)
    if not marked_workspace:
        print("catkin clean: error: Could not clean workspace \"%s\" because it "
              "either does not exist or it has no catkin_tools metadata." %
              opts.workspace)
        return 1

    # Get the catkin build spaces
    build_metadata = metadata.get_metadata(marked_workspace, 'build')

    # Metadata group
    if opts.metadata:
        # Delete the metadata
        metadata.init_metadata_dir(
            marked_workspace,
            reset=True)
    elif opts.metadata_dir:
        # Delete the metadata directory
        (metadata_dir, _) = metadata.get_paths(marked_workspace)
        print("Deleting metadata directory: %s" % metadata_dir)
        shutil.rmtree(metadata_dir)

    # Check the build metadata
    if not build_metadata:
        print("catkin clean: error: Could not clean workspace \"%s\" because it "
              "it is missing the catkin build metadata." % marked_workspace)
        return 1

    # Remove the build and develspaces
    build_space = os.path.join(marked_workspace, build_metadata['build_space'])
    devel_space = os.path.join(marked_workspace, build_metadata['devel_space'])
    source_space = os.path.join(marked_workspace, build_metadata['source_space'])

    # Only group (destruction-limiting)
    if opts.ccache_only:
        # Clear the CMakeCache for each package
        if not os.path.exists(build_space):
            print("No buildspace exists, no CMake caches to clear.")
            return 0

        # Remove CMakeCaches
        print("Removing CMakeCache.txt files from %s" % build_space)
        for pkg_build_name in os.listdir(build_space):
            if pkg_build_name not in exempt_build_dirs:
                pkg_build_path = os.path.join(build_space, pkg_build_name)
                ccache_path = os.path.join(pkg_build_path, 'CMakeCache.txt')

                if os.path.exists(ccache_path):
                    print(" - Removing %s" % ccache_path)
                    os.remove(ccache_path)

    elif opts.orphans_only:
        if not os.path.exists(build_space):
            print("No buildspace exists, no potential for orphans.")
            return 0

        # TODO: Check for merged build and report error
        # Get all enabled packages in source space
        found_source_packages = [pkg.name for (path, pkg) in find_packages(source_space).items()]

        # Iterate over all packages with build dirs
        print("Removing orphaned build directories from %s" % build_space)
        no_orphans = True
        for pkg_build_name in os.listdir(build_space):
            if pkg_build_name not in exempt_build_dirs:
                pkg_build_path = os.path.join(build_space, pkg_build_name)
                # Remove package build dir if not found
                if pkg_build_name not in found_source_packages:
                    no_orphans = False
                    print(" - Removing %s" % pkg_build_path)
                    shutil.rmtree(pkg_build_path)

        if no_orphans:
            print(" - No orphans found, nothing removed from buildspace.")
        else:
            # Remove the develspace
            # TODO: For isolated devel, this could just remove individual packages
            if os.path.exists(devel_space):
                print("Removing develspace: %s" % devel_space)
                shutil.rmtree(devel_space)
    elif opts.devel_only:
        if os.path.exists(devel_space):
            print("Removing develspace: %s" % devel_space)
            shutil.rmtree(devel_space)
    else:
        # Remove the buildspace and develspace
        if os.path.exists(build_space):
            print("Removing buildspace: %s" % build_space)
            shutil.rmtree(build_space)
        if os.path.exists(devel_space):
            print("Removing develspace: %s" % devel_space)
            shutil.rmtree(devel_space)

    return 0
