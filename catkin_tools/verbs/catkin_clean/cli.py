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
import sys

from catkin_pkg.packages import find_packages

from catkin_tools.argument_parsing import add_context_args

from catkin_tools.context import Context

from catkin_tools.jobs.job import get_build_type

from catkin_tools.metadata import update_metadata

from catkin_tools.terminal_color import ColorMapper

from .clean import determine_packages_to_be_cleaned
from .clean import clean_packages

color_mapper = ColorMapper()
clr = color_mapper.clr

# Exempt build directories
# See https://github.com/catkin/catkin_tools/issues/82
exempt_build_files = ['build_logs', 'clean_logs', '.catkin_tools.yaml']

setup_files = ['.catkin', 'env.sh', 'setup.bash', 'setup.sh', 'setup.zsh', '_setup_util.py']


def prepare_arguments(parser):
    # Workspace / profile args
    add_context_args(parser)

    # Basic group
    basic_group = parser.add_argument_group('Basic', 'Clean workspace subdirectories.')
    add = basic_group.add_argument
    add('-a', '--all', action='store_true', default=False,
        help='Remove all of the *spaces associated with the given or active'
        ' profile. This will remove everything but the source space and the'
        ' hidden .catkin_tools directory.')
    add('-b', '--build', action='store_true', default=False,
        help='Remove the buildspace.')
    add('-d', '--devel', action='store_true', default=False,
        help='Remove the develspace.')
    add('-i', '--install', action='store_true', default=False,
        help='Remove the installspace.')

    # Advanced group
    advanced_group = parser.add_argument_group(
        'Advanced',
        "Clean only specific parts of the workspace. These options will "
        "automatically enable the --force-cmake option for the next build "
        "invocation.")
    add = advanced_group.add_argument
    add('-c', '--cmake-cache', action='store_true', default=False,
        help='Clear the CMakeCache for each package, but leave build and devel spaces.')

    add('-s', '--setup-files', action='store_true', default=False,
        help='Clear the catkin-generated files in order to rebase onto another workspace.')

    add('-o', '--orphans', action='store_true', default=False,
        help='Remove products from source packages are no'
        ' longer enabled or in the source space. This might require'
        ' --force-cmake on the next build.')

    add('--no-deps', action='store_true', default=False,
        help='Don\'t clean the packages which depend on the packages to be cleaned.')

    add('packages', metavar='PKGNAME', nargs='*',
        help='Workspace packages to clean. This will clean the given packages'
        ' and the packages which depend on them.')

    return parser


def main(opts):
    actions = ['all', 'build', 'devel', 'install', 'cmake_cache', 'orphans', 'setup_files']
    if not any([v for (k, v) in vars(opts).items() if k in actions]):
        print("[clean] No actions performed. See `catkin clean -h` for usage.")
        return 0

    needs_force = False

    # Load the context
    ctx = Context.load(opts.workspace, opts.profile, opts, strict=True, load_env=False)

    if not ctx:
        if not opts.workspace:
            print(
                "catkin clean: error: The current or desired workspace could not be "
                "determined. Please run `catkin clean` from within a catkin "
                "workspace or specify the workspace explicitly with the "
                "`--workspace` option.")
        else:
            print(
                "catkin clean: error: Could not clean workspace \"%s\" because it "
                "either does not exist or it has no catkin_tools metadata." %
                opts.workspace)
        return 1

    # Remove the requested spaces
    if opts.all:
        opts.build = opts.devel = opts.install = True

    # Orphan removal
    orphans = []
    if opts.orphans:
        if os.path.exists(ctx.build_space_abs):
            # TODO: Check for merged build and report error

            # Get all enabled packages in source space
            # Suppress warnings since this is looking for packages which no longer exist
            found_source_packages = [
                pkg.name for (path, pkg) in find_packages(ctx.source_space_abs, warnings=[]).items()]

            # Iterate over all packages with build dirs
            print("[clean] Removing orphaned build directories from %s" % ctx.build_space_abs)
            for pkg_build_name in os.listdir(ctx.build_space_abs):
                if pkg_build_name not in exempt_build_files:
                    pkg_build_path = os.path.join(ctx.build_space_abs, pkg_build_name)
                    # Remove package build dir if not found
                    if pkg_build_name not in found_source_packages:
                        orphans.append(pkg_build_name)

            if len(orphans) > 0:
                print("[clean] Removing orphaned packages: ")
                for pkg_name in orphans:
                    print("[clean] - %s" % pkg_name)

                opts.packages.extend(orphans)
            else:
                sys.exit("No orphans in the workspace.")
        else:
            print("[clean] No buildspace exists, no potential for orphans.")

    # Remove specific packages
    if len(opts.packages) > 0:

        packages_to_be_cleaned, packages_to_be_cleaned_dependants, ordered_packages = determine_packages_to_be_cleaned(
            opts.packages + orphans, ctx)

        if not opts.no_deps:
            packages_to_be_cleaned.extend(packages_to_be_cleaned_dependants)

        package_names_to_be_cleaned = [pkg.name for _, pkg in packages_to_be_cleaned]

        clean_packages(ctx, packages_to_be_cleaned, opts.build, opts.devel, opts.install)
    else:
        # Remove al buildspace files
        if opts.build:
            if os.path.exists(ctx.build_space_abs):
                print("[clean] Removing buildspace: %s" % ctx.build_space_abs)
                shutil.rmtree(ctx.build_space_abs)

        # Remove all installspace files
        if opts.install:
            if os.path.exists(ctx.install_space_abs):
                print("[clean] Removing installspace: %s" % ctx.install_space_abs)
                shutil.rmtree(ctx.install_space_abs)

        # Remove all develspace files
        if opts.devel:
            if os.path.exists(ctx.devel_space_abs):
                print("[clean] Removing develspace: %s" % ctx.devel_space_abs)
                shutil.rmtree(ctx.devel_space_abs)

    # Setup file removal
    if opts.setup_files:
        print("[clean] Removing setup files from develspace: %s" % ctx.devel_space_abs)
        for filename in setup_files:
            full_path = os.path.join(ctx.devel_space_abs, filename)
            if os.path.exists(full_path):
                print(" - Removing %s" % full_path)
                os.remove(full_path)
                needs_force = True

    # CMake Cache removal
    if opts.cmake_cache:
        # Clear the CMakeCache for each package
        if os.path.exists(ctx.build_space_abs):
            # Remove CMakeCaches
            print("[clean] Removing CMakeCache.txt files from %s" % ctx.build_space_abs)
            for pkg_build_name in os.listdir(ctx.build_space_abs):
                if pkg_build_name not in exempt_build_files:
                    pkg_build_path = os.path.join(ctx.build_space_abs, pkg_build_name)
                    ccache_path = os.path.join(pkg_build_path, 'CMakeCache.txt')

                    if os.path.exists(ccache_path):
                        print(" - Removing %s" % ccache_path)
                        os.remove(ccache_path)
                        needs_force = True
        else:
            print("[clean] No buildspace exists, no CMake caches to clear.")

    if needs_force:
        print(
            "NOTE: Parts of the workspace have been cleaned which will "
            "necessitate re-configuring CMake on the next build.")
        update_metadata(ctx.workspace, ctx.profile, 'build', {'needs_force': True})

    return 0
