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

try:
    raw_input
except NameError:
    raw_input = input

import os
import shutil

from catkin_pkg.packages import find_packages

from catkin_tools.argument_parsing import add_context_args

from catkin_tools.context import Context

from catkin_tools.common import log

import catkin_tools.execution.job_server as job_server

from catkin_tools.metadata import update_metadata
from catkin_tools.metadata import METADATA_DIR_NAME

from catkin_tools.terminal_color import ColorMapper

from .clean import clean_packages

color_mapper = ColorMapper()
clr = color_mapper.clr

# Exempt build directories
# See https://github.com/catkin/catkin_tools/issues/82
exempt_build_files = ['.catkin_tools.yaml', 'catkin_tools_prebuild']

setup_files = ['.catkin', 'env.sh', 'setup.bash', 'setup.sh', 'setup.zsh', '_setup_util.py']


def yes_no_loop(question):
    while True:
        resp = str(raw_input(question + " [yN]: "))
        if resp.lower() in ['n', 'no'] or len(resp) == 0:
            return False
        elif resp.lower() in ['y', 'yes']:
            return True
        log(clr("[clean] Please answer either \"yes\" or \"no\"."))


def prepare_arguments(parser):
    # Workspace / profile args
    add_context_args(parser)

    add = parser.add_argument
    add('--dry-run', '-n', action='store_true', default=False,
        help='Show the effects of the clean action without modifying the workspace.')
    add('--verbose', '-v', action='store_true', default=False,
        help='Verbose status output.')
    add('--force', '-f', action='store_true', default=False,
        help='Skip all interactive checks.')

    # Basic group
    basic_group = parser.add_argument_group('Basic', 'Clean workspace subdirectories.')
    add = basic_group.add_argument
    add('-a', '--all', action='store_true', default=False,
        help='Remove all of the generated spaces associated with the given or '
        'active profile. This will remove everything but the source space and '
        'the hidden .catkin_tools directory.')
    add('-b', '--build', action='store_true', default=False,
        help='Remove the entire buildspace.')
    add('-d', '--devel', action='store_true', default=False,
        help='Remove the entire develspace.')
    add('-i', '--install', action='store_true', default=False,
        help='Remove the entire installspace.')
    add('--deinit', action='store_true', default=False,
        help='De-initialize the workspace, delete all build profiles and configuration.')

    # Packages group
    packages_group = parser.add_argument_group(
        'Packages',
        "Clean products from specific packages in the workspace. These options will "
        "automatically enable the --force-cmake option for the next build "
        "invocation.")
    add = packages_group.add_argument
    add('packages', metavar='PKGNAME', nargs='*',
        help='Explicilty specify a list of specific packages to clean from the build, devel, and install space.')
    add('--deps', action='store_true', default=False,
        help='Clean the packages which depend on other packages to be cleaned.')
    add('--orphans', action='store_true', default=False,
        help='Remove products from packages are no longer in the source space. '
        'Note that this also removes packages which are '
        'blacklisted or which contain `CATKIN_INGORE` marker files.')

    # Advanced group
    advanced_group = parser.add_argument_group(
        'Advanced',
        "Clean only specific parts of the workspace for specified packages. These options will "
        "automatically enable the --force-cmake option for the next build "
        "invocation.")
    add = advanced_group.add_argument
    add('-l', '--logs', action='store_true', default=False,
        help='Only clear the catkin-generated logfiles.')
    add('--setup-files', action='store_true', default=False,
        help='Clear the catkin-generated setup files from the devel and install spaces.')

    return parser


def main(opts):
    # Load the context
    ctx = Context.load(opts.workspace, opts.profile, opts, strict=True, load_env=False)

    if not ctx:
        if not opts.workspace:
            print(
                "[clean] Error: The current or desired workspace could not be "
                "determined. Please run `catkin clean` from within a catkin "
                "workspace or specify the workspace explicitly with the "
                "`--workspace` option.")
        else:
            print(
                "[clean] Error: Could not clean workspace \"%s\" because it "
                "either does not exist or it has no catkin_tools metadata." %
                opts.workspace)
        return 1

    # Initialize job server
    job_server.initialize(
        max_jobs=1,
        max_load=None,
        gnu_make_enabled=False)

    # Check if the user wants to do something explicit
    actions = ['all', 'build', 'devel', 'install', 'deinit', 'orphans',
               'setup_files', 'packages', 'logs']

    build_exists = os.path.exists(ctx.build_space_abs)
    devel_exists = os.path.exists(ctx.devel_space_abs)
    install_exists = os.path.exists(ctx.install_space_abs)

    if not any([build_exists, devel_exists, install_exists]) and not opts.deinit:
        log("[clean] Neither the build, devel, or install spaces for this "
            "profile exist. Nothing to be done.")
        return 0

    # Default is to clean all products for this profile
    if not any([v for (k, v) in vars(opts).items() if k in actions]):
        opts.all = True

    # Make sure the user intends to clena everything
    if opts.all and not (opts.force or opts.dry_run):
        log(clr("[clean] @!@{yf}Warning:@| This will completely remove the "
                "existing build, devel, and install spaces for this profile. "
                "Use `--force` to skip this check."))
        if build_exists:
            log(clr("[clean]   Build Space:   @{yf}{}").format(ctx.build_space_abs))
        if devel_exists:
            log(clr("[clean]   Devel Space:   @{yf}{}").format(ctx.devel_space_abs))
        if install_exists:
            log(clr("[clean]   Install Space: @{yf}{}").format(ctx.install_space_abs))

        try:
            opts.all = yes_no_loop("\n[clean] Are you sure you want to completely remove the directories listed above?")
            if not opts.all:
                log(clr("[clean] Not removing build, devel, or install spaces for this profile."))
        except KeyboardInterrupt:
            log("\n[clean] No actions performed.")
            return 0

    # Warn before nuking .catkin_tools
    if opts.deinit and not opts.force:
        log(clr("[clean] @!@{yf}Warning:@| If you deinitialize this workspace "
                "you will lose all profiles and all saved build configuration. "
                "Use `--force` to skip this check."))
        try:
            opts.deinit = yes_no_loop("\n[clean] Are you sure you want to deinitialize this workspace?")
            if not opts.deinit:
                log(clr("[clean] Not deinitializing workspace."))
        except KeyboardInterrupt:
            log("\n[clean] No actions performed.")
            return 0

    # Initialize flag to be used on the next invocation
    needs_force = False

    # Remove the requested spaces
    if opts.all:
        opts.build = opts.devel = opts.install = True

    try:
        # Remove all installspace files
        if opts.install:
            if os.path.exists(ctx.install_space_abs):
                print("[clean] Removing installspace: %s" % ctx.install_space_abs)
                if not opts.dry_run:
                    shutil.rmtree(ctx.install_space_abs)

        # Remove all develspace files
        if opts.devel:
            if os.path.exists(ctx.devel_space_abs):
                print("[clean] Removing develspace: %s" % ctx.devel_space_abs)
                if not opts.dry_run:
                    shutil.rmtree(ctx.devel_space_abs)

        # Remove all buildspace files
        if opts.build:
            if os.path.exists(ctx.build_space_abs):
                print("[clean] Removing buildspace: %s" % ctx.build_space_abs)
                if not opts.dry_run:
                    shutil.rmtree(ctx.build_space_abs)

        # Find orphaned packages
        if ctx.link_devel and not any([opts.build, opts.devel]):
            if opts.orphans:
                if os.path.exists(ctx.build_space_abs):
                    # Initialize orphan list
                    orphans = set()

                    # Get all enabled packages in source space
                    # Suppress warnings since this is looking for packages which no longer exist
                    found_source_packages = [
                        pkg.name for (path, pkg) in find_packages(ctx.source_space_abs, warnings=[]).items()]

                    # Look for orphaned products in the build space
                    print("[clean] Determining orphaned packages...")
                    for pkg_build_name in os.listdir(ctx.build_space_abs):
                        if pkg_build_name not in exempt_build_files:
                            if pkg_build_name not in found_source_packages:
                                orphans.add(pkg_build_name)
                                print("[clean] Orphaned build products: %s" % pkg_build_name)

                    # Look for orphaned products in the develspace
                    for pkg_devel_name in ctx.private_devel_path():
                        if os.path.isdir(pkg_devel_name):
                            if pkg_devel_name not in found_source_packages:
                                orphans.add(pkg_devel_name)
                                print("[clean] Orphaned devel products: %s" % pkg_devel_name)

                    if len(orphans) > 0:
                        opts.packages.extend(list(orphans))
                    else:
                        print("[clean] No orphans in the workspace.")
                else:
                    print("[clean] No buildspace exists, no potential for orphans.")

            # Remove specific packages
            if len(opts.packages) > 0:

                try:
                    # Clean the packages
                    needs_force = clean_packages(
                        ctx,
                        opts.packages,
                        opts.deps,
                        opts.verbose,
                        opts.dry_run)
                except KeyboardInterrupt:
                    wide_log("[build] User interrupted!")
                    event_queue.put(None)

        elif opts.orphans or len(opts.packages) > 0:
            print("[clean] Error: Individual packages can only be cleaned from "
                  "workspaces with symbolically-linked develspaces (`catkin "
                  "config --link-devel`).")

        # Clean log files
        if opts.logs:
            log_dir = os.path.join(ctx.metadata_path(), 'logs')
            if os.path.exists(log_dir):
                print("[clean] Removing log files from: {}".format(log_dir))
                if not opts.dry_run:
                    shutil.rmtree(log_dir)
            else:
                print("[clean] Log file directory does not exist: {}".format(log_dir))

        # Nuke .catkin_tools
        if opts.deinit:
            metadata_dir = os.path.join(ctx.workspace, METADATA_DIR_NAME)
            print("[clean] Deinitializing workspace by removing catkin_tools config: %s" % metadata_dir)
            if not opts.dry_run:
                shutil.rmtree(metadata_dir)

        # Setup file removal
        if opts.setup_files:
            if os.path.exists(ctx.devel_space_abs):
                print("[clean] Removing setup files from develspace: %s" % ctx.devel_space_abs)
                for filename in setup_files:
                    full_path = os.path.join(ctx.devel_space_abs, filename)
                    if os.path.exists(full_path):
                        print(" - Removing %s" % full_path)
                        os.remove(full_path)
                        needs_force = True
            else:
                print("[clean] No develspace exists, no setup files to clean.")
    except:
        needs_force = True
        raise

    finally:
        if needs_force:
            print(clr(
                "[clean] @/@!Note:@| @/Parts of the workspace have been cleaned which will "
                "necessitate re-configuring CMake on the next build.@|"))
            update_metadata(ctx.workspace, ctx.profile, 'build', {'needs_force': True})

    return 0
