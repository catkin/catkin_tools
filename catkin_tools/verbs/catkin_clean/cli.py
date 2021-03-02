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

import os
import shutil
import sys

from catkin_pkg.package import InvalidPackage
from catkin_pkg.packages import find_packages

from catkin_tools.argument_parsing import add_context_args

from catkin_tools.context import Context

from catkin_tools.common import log
from catkin_tools.common import wide_log
from catkin_tools.common import find_enclosing_package
from catkin_tools.common import getcwd

import catkin_tools.execution.job_server as job_server

from catkin_tools.metadata import find_enclosing_workspace
from catkin_tools.metadata import get_paths as get_metadata_paths
from catkin_tools.metadata import get_profile_names
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
        resp = str(input(question + " [yN]: "))
        if resp.lower() in ['n', 'no'] or len(resp) == 0:
            return False
        elif resp.lower() in ['y', 'yes']:
            return True
        log(clr("[clean] Please answer either \"yes\" or \"no\"."))


def safe_rmtree(path, workspace_root, force):
    """Safely remove a path outside of the workspace root."""

    # Check if the path is inside the workspace
    path_in_workspace = path.find(workspace_root) == 0

    yes = True
    if not path_in_workspace and not force:
        log(clr("[clean] Warning: `{}` is outside of the workspace root. (Use"
                " --force to skip this check)".format(path)))
        yes = yes_no_loop("Are you sure you want to entirely remove `{}`?".format(path))

    if yes:
        shutil.rmtree(path)
    else:
        log("[clean] Not removing `{}`".format(path))


def prepare_arguments(parser):
    # Workspace / profile args
    add_context_args(parser)

    add = parser.add_argument
    add('--dry-run', '-n', action='store_true', default=False,
        help='Show the effects of the clean action without modifying the workspace.')
    add('--verbose', '-v', action='store_true', default=False,
        help='Verbose status output.')
    add('--yes', '-y', action='store_true', default=False,
        help='Assume "yes" to all interactive checks.')
    add('--force', '-f', action='store_true', default=False,
        help='Allow cleaning files outside of the workspace root.')
    add('--all-profiles', action='store_true', default=False,
        help='Apply the specified clean operation for all profiles in this workspace.')

    full_group = parser.add_argument_group(
        'Full',
        'Remove everything except the source space.')
    add = full_group.add_argument
    add('--deinit', action='store_true', default=False,
        help='De-initialize the workspace, delete all build profiles and'
        ' configuration. This will also clean subdirectories for all profiles in'
        ' the workspace.')

    # Basic group
    basic_group = parser.add_argument_group(
        'Spaces',
        'Clean workspace subdirectories for the selected profile.')
    add = basic_group.add_argument
    add('-l', '--logs', action='store_true', default=False,
        help='Remove the entire log space.')
    add('-b', '--build', action='store_true', default=False,
        help='Remove the entire build space.')
    add('-d', '--devel', action='store_true', default=False,
        help='Remove the entire devel space.')
    add('-i', '--install', action='store_true', default=False,
        help='Remove the entire install space.')

    # Packages group
    packages_group = parser.add_argument_group(
        'Packages',
        "Clean products from specific packages in the workspace. Note that"
        " these options are only available in a `linked` devel space layout."
        " These options will also automatically enable the --force-cmake"
        " option for the next build invocation.")
    add = packages_group.add_argument
    add('packages', metavar='PKGNAME', nargs='*',
        help='Explicilty specify a list of specific packages to clean from the build, devel, and install space.')
    add('--this', dest='clean_this', action='store_true', default=False,
        help='Clean the package containing the current working directory from the build, devel, and install space.')
    add('--dependents', '--deps', action='store_true', default=False,
        help='Clean the packages which depend on the packages to be cleaned.')
    add('--orphans', action='store_true', default=False,
        help='Remove products from packages are no longer in the source space. '
        'Note that this also removes packages which are '
        'blacklisted or which contain `CATKIN_IGNORE` marker files.')

    # Advanced group
    advanced_group = parser.add_argument_group(
        'Advanced',
        "Clean other specific parts of the workspace.")
    add = advanced_group.add_argument
    add('--setup-files', action='store_true', default=False,
        help='Clear the catkin-generated setup files from the devel and install spaces.')

    return parser


def clean_profile(opts, profile):
    # Load the context
    ctx = Context.load(opts.workspace, profile, opts, strict=True, load_env=False)

    if not ctx:
        if not opts.workspace:
            log(
                "[clean] Error: The current or desired workspace could not be "
                "determined. Please run `catkin clean` from within a catkin "
                "workspace or specify the workspace explicitly with the "
                "`--workspace` option.")
        else:
            log(
                "[clean] Error: Could not clean workspace \"%s\" because it "
                "either does not exist or it has no catkin_tools metadata." %
                opts.workspace)
        return False

    profile = ctx.profile

    # Check if the user wants to do something explicit
    actions = [
        'build', 'devel', 'install', 'logs',
        'packages', 'clean_this', 'orphans',
        'deinit',  'setup_files']

    logs_exists = os.path.exists(ctx.log_space_abs)
    build_exists = os.path.exists(ctx.build_space_abs)
    devel_exists = os.path.exists(ctx.devel_space_abs)

    install_path = (
        os.path.join(ctx.destdir, ctx.install_space_abs.lstrip(os.sep))
        if ctx.destdir
        else ctx.install_space_abs)
    install_exists = os.path.exists(install_path)

    # Default is to clean all products for this profile
    no_specific_action = not any([
        v for (k, v) in vars(opts).items()
        if k in actions])
    clean_all = opts.deinit or no_specific_action

    # Initialize action options
    if clean_all:
        opts.logs = opts.build = opts.devel = opts.install = True

    # Make sure the user intends to clena everything
    spaces_to_clean = (opts.logs or opts.build or opts.devel or opts.install)
    spaces_to_clean_msgs = []

    if spaces_to_clean and not (opts.yes or opts.dry_run):
        if opts.logs and logs_exists:
            spaces_to_clean_msgs.append(clr("[clean] Log Space:     @{yf}{}").format(ctx.log_space_abs))
        if opts.build and build_exists:
            spaces_to_clean_msgs.append(clr("[clean] Build Space:   @{yf}{}").format(ctx.build_space_abs))
        if opts.devel and devel_exists:
            spaces_to_clean_msgs.append(clr("[clean] Devel Space:   @{yf}{}").format(ctx.devel_space_abs))
        if opts.install and install_exists:
            spaces_to_clean_msgs.append(clr("[clean] Install Space: @{yf}{}").format(install_path))

        if len(spaces_to_clean_msgs) == 0 and not opts.deinit:
            log("[clean] Nothing to be cleaned for profile:  `{}`".format(profile))
            return True

    if len(spaces_to_clean_msgs) > 0:
        log("")
        log(clr("[clean] @!@{yf}Warning:@| This will completely remove the "
                "following directories. (Use `--yes` to skip this check)"))
        for msg in spaces_to_clean_msgs:
            log(msg)
        try:
            yes = yes_no_loop(
                "\n[clean] Are you sure you want to completely remove the directories listed above?")
            if not yes:
                log(clr("[clean] Not removing any workspace directories for"
                        " this profile."))
                return True
        except KeyboardInterrupt:
            log("\n[clean] No actions performed.")
            sys.exit(0)

    # Initialize flag to be used on the next invocation
    needs_force = False

    try:
        # Remove all installspace files
        if opts.install and install_exists:
            log("[clean] Removing installspace: %s" % install_path)
            if not opts.dry_run:
                safe_rmtree(install_path, ctx.workspace, opts.force)

        # Remove all develspace files
        if opts.devel:
            if devel_exists:
                log("[clean] Removing develspace: %s" % ctx.devel_space_abs)
                if not opts.dry_run:
                    safe_rmtree(ctx.devel_space_abs, ctx.workspace, opts.force)
            # Clear the cached metadata from the last build run
            _, build_metadata_file = get_metadata_paths(ctx.workspace, profile, 'build')
            if os.path.exists(build_metadata_file):
                os.unlink(build_metadata_file)
            # Clear the cached packages data, if it exists
            packages_metadata_path = ctx.package_metadata_path()
            if os.path.exists(packages_metadata_path):
                safe_rmtree(packages_metadata_path, ctx.workspace, opts.force)

        # Remove all buildspace files
        if opts.build and build_exists:
            log("[clean] Removing buildspace: %s" % ctx.build_space_abs)
            if not opts.dry_run:
                safe_rmtree(ctx.build_space_abs, ctx.workspace, opts.force)

        # Setup file removal
        if opts.setup_files:
            if devel_exists:
                log("[clean] Removing setup files from develspace: %s" % ctx.devel_space_abs)
                opts.packages.append('catkin')
                opts.packages.append('catkin_tools_prebuild')
            else:
                log("[clean] No develspace exists, no setup files to clean.")

        # Clean log files
        if opts.logs and logs_exists:
            log("[clean] Removing log space: {}".format(ctx.log_space_abs))
            if not opts.dry_run:
                safe_rmtree(ctx.log_space_abs, ctx.workspace, opts.force)

        # Find orphaned packages
        if ctx.link_devel or ctx.isolate_devel and not any([opts.build, opts.devel]):
            if opts.orphans:
                if os.path.exists(ctx.build_space_abs):
                    log("[clean] Determining orphaned packages...")

                    # Get all existing packages in source space and the
                    # Suppress warnings since this is looking for packages which no longer exist
                    found_source_packages = [
                        pkg.name for (path, pkg) in
                        find_packages(ctx.source_space_abs, warnings=[]).items()]
                    built_packages = [
                        pkg.name for (path, pkg) in
                        find_packages(ctx.package_metadata_path(), warnings=[]).items()]

                    # Look for orphaned products in the build space
                    orphans = [p for p in built_packages
                               if (p not in found_source_packages and p !=
                                   'catkin_tools_prebuild')]

                    if len(orphans) > 0:
                        opts.packages.extend(list(orphans))
                    else:
                        log("[clean] No orphans in the workspace.")
                else:
                    log("[clean] No buildspace exists, no potential for orphans.")

            # Remove specific packages
            if len(opts.packages) > 0 or opts.clean_this:
                # Determine the enclosing package
                try:
                    ws_path = find_enclosing_workspace(getcwd())
                    # Suppress warnings since this won't necessaraly find all packages
                    # in the workspace (it stops when it finds one package), and
                    # relying on it for warnings could mislead people.
                    this_package = find_enclosing_package(
                        search_start_path=getcwd(),
                        ws_path=ws_path,
                        warnings=[])
                except InvalidPackage as ex:
                    sys.exit(clr("@{rf}Error:@| The file %s is an invalid package.xml file."
                                 " See below for details:\n\n%s" % (ex.package_path, ex.msg)))

                # Handle context-based package cleaning
                if opts.clean_this:
                    if this_package:
                        opts.packages += [this_package]
                    else:
                        sys.exit(
                            "[clean] Error: In order to use --this, the current directory"
                            " must be part of a catkin package.")
                try:
                    # Clean the packages
                    needs_force = clean_packages(
                        ctx,
                        opts.packages,
                        opts.dependents,
                        opts.verbose,
                        opts.dry_run)
                except KeyboardInterrupt:
                    wide_log("[clean] User interrupted!")
                    return False

        elif opts.orphans or len(opts.packages) > 0 or opts.clean_this:
            log("[clean] Error: Individual packages cannot be cleaned from "
                "workspaces with merged develspaces, use a symbolically-linked "
                "or isolated develspace instead.")

    except:  # noqa: E722
        # Silencing E722 here since we immediately re-raise the exception.
        log("[clean] Failed to clean profile `{}`".format(profile))
        needs_force = True
        raise

    finally:
        if needs_force:
            log(clr(
                "[clean] @/@!Note:@| @/Parts of the workspace have been cleaned which will "
                "necessitate re-configuring CMake on the next build.@|"))
            update_metadata(ctx.workspace, ctx.profile, 'build', {'needs_force': True})

    return True


def main(opts):
    # Check for exclusivity
    full_options = opts.deinit
    space_options = opts.logs or opts.build or opts.devel or opts.install
    package_options = len(opts.packages) > 0 or opts.orphans or opts.clean_this
    advanced_options = opts.setup_files

    if full_options:
        if space_options or package_options or advanced_options:
            log("[clean] Error: Using `--deinit` will remove all spaces, so"
                " additional partial cleaning options will be ignored.")
    elif space_options:
        if package_options:
            log("[clean] Error: Package arguments are not allowed with space"
                " arguments (--build, --devel, --install, --logs). See usage.")
        elif advanced_options:
            log("[clean] Error: Advanced arguments are not allowed with space"
                " arguments (--build, --devel, --install, --logs). See usage.")

    # Check for all profiles option
    if opts.all_profiles:
        profiles = get_profile_names(opts.workspace or find_enclosing_workspace(getcwd()))
    else:
        profiles = [opts.profile]

    # Initialize job server
    job_server.initialize(
        max_jobs=1,
        max_load=None,
        gnu_make_enabled=False)

    # Clean the requested profiles
    retcode = 0
    for profile in profiles:
        if not clean_profile(opts, profile):
            retcode = 1

    # Warn before nuking .catkin_tools
    if retcode == 0:
        if opts.deinit and not opts.yes:
            log("")
            log(clr("[clean] @!@{yf}Warning:@| If you deinitialize this workspace"
                    " you will lose all profiles and all saved build"
                    " configuration. (Use `--yes` to skip this check)"))
            try:
                opts.deinit = yes_no_loop("\n[clean] Are you sure you want to deinitialize this workspace?")
                if not opts.deinit:
                    log(clr("[clean] Not deinitializing workspace."))
            except KeyboardInterrupt:
                log("\n[clean] No actions performed.")
                sys.exit(0)

        # Nuke .catkin_tools
        if opts.deinit:
            ctx = Context.load(opts.workspace, profile, opts, strict=True, load_env=False)
            metadata_dir = os.path.join(ctx.workspace, METADATA_DIR_NAME)
            log("[clean] Deinitializing workspace by removing catkin_tools config: %s" % metadata_dir)
            if not opts.dry_run:
                safe_rmtree(metadata_dir, ctx.workspace, opts.force)

    return retcode
