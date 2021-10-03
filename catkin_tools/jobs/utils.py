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

from catkin_tools.common import mkdir_p
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace

from catkin_tools.resultspace import get_resultspace_environment

from catkin_tools.execution.events import ExecutionEvent


class CommandMissing(Exception):
    '''A required command is missing.'''

    def __init__(self, name):
        super(CommandMissing, self).__init__(
                'Cannot find required tool `%s` on the PATH, is it installed?' % name)


def require_command(name, which):
    if not which:
        raise CommandMissing(name)


def get_env_loaders(package, context):
    """Get a list of env loaders required to build this package."""

    sources = []
    # If installing to isolated folders or not installing, but devel spaces are not merged
    if (context.install and context.isolate_install) or (not context.install and context.isolate_devel):
        # Source each package's install or devel space
        space = context.install_space_abs if context.install else context.devel_space_abs
        # Get the recursive dependcies
        depends = get_cached_recursive_build_depends_in_workspace(package, context.packages)
        # For each dep add a line to source its setup file
        for dep_pth, dep in depends:
            source_path = os.path.join(space, dep.name, 'env.sh')
            sources.append(source_path)
    else:
        # Get the actual destination of this package
        if context.link_devel and not context.install:
            source_path = os.path.join(context.package_final_path(package), 'env.sh')
        else:
            source_path = os.path.join(context.package_dest_path(package), 'env.sh')
        sources = [source_path]

    return sources


def merge_envs(job_env, overlay_envs):
    '''
    In the merged/linked case of single env, this function amounts to a straight
    assignment, but a more complex merge is required with isolated result spaces,
    since a package's build environment may require extending that of multiple
    other result spaces.
    '''
    merge_path_values = {}

    for overlay_env in overlay_envs:
        for key, values_str in overlay_env.items():
            if key.endswith('PATH'):
                if key not in merge_path_values:
                    # Seed the list with any values already in the environment. We reverse the list
                    # here so that we can cheaply append to it, representing a prepend in the final
                    # PATH var, and because we need to maintain the order of underlay paths.
                    if key in job_env:
                        values = job_env[key].split(os.pathsep)
                        values.reverse()
                        merge_path_values[key] = values
                    else:
                        merge_path_values[key] = []
                merge_path_values[key].extend(values_str.split(os.pathsep))
            else:
                # For non-PATH keys, simply assign the value. This may not always
                # be correct behaviour, but we don't have the information here to
                # know how to do anything else.
                job_env[key] = values_str

    # For the path values, do a deduplicating merge.
    for key, values_list in merge_path_values.items():
        seen_values = set()
        new_values_list = []
        for value in values_list:
            if value not in seen_values:
                seen_values.add(value)
                new_values_list.append(value)
        job_env[key] = os.pathsep.join(reversed(new_values_list))


def loadenv(logger, event_queue, job_env, package, context, verbose=True):
    # Get the paths to the env loaders
    env_loader_paths = get_env_loaders(package, context)
    # If DESTDIR is set, set _CATKIN_SETUP_DIR as well
    if context.destdir is not None:
        job_env['_CATKIN_SETUP_DIR'] = context.package_dest_path(package)

    envs = []
    for env_loader_path in env_loader_paths:
        if logger and verbose:
            logger.out('Loading environment from: {}'.format(env_loader_path))
        envs.append(get_resultspace_environment(
            os.path.split(env_loader_path)[0],
            base_env=job_env,
            quiet=True,
            cached=context.use_env_cache,
            strict=False))

    # Avoid using merge logic if not required (in the non-isolated resultspace
    # case. It has corner cases which may trip up the unwary, so having the
    # option to switch to a merged resultspace is a good fallback.
    if len(envs) > 1:
        merge_envs(job_env, envs)
    elif len(envs) == 1:
        job_env.update(envs[0])
    return 0


def makedirs(logger, event_queue, path):
    """FunctionStage functor that makes a path of directories."""
    mkdir_p(path)
    return 0


def copyfiles(logger, event_queue, source_paths, dest_path):
    """FunctionStage functor that copies one or more files"""
    for source_path in source_paths:
        shutil.copy(source_path, dest_path)
    return 0


def rmfile(logger, event_queue, path):
    """FunctionStage functor that removes a file."""
    if os.path.exists(path):
        os.remove(path)
    return 0


def rmdirs(logger, event_queue, paths):
    """FunctionStage functor that removes a directory tree."""
    return rmfiles(logger, event_queue, paths, remove_empty=False)


def rmfiles(logger, event_queue, paths, dry_run, remove_empty=False, empty_root='/'):
    """FunctionStage functor that removes a list of files and directories.

    If remove_empty is True, then this will also remove directories which
    become emprt after deleting the files in `paths`. It will delete files up
    to the path specified by `empty_root`.
    """

    # Determine empty directories
    if remove_empty:
        # First get a list of directories to check
        dirs_to_check = set()

        for path in paths:
            # Make sure the file is given by an absolute path and it exists
            if not os.path.isabs(path) or not os.path.exists(path):
                continue

            # Only look in the devel space
            while empty_root.find(path) != 0:
                # Pop up a directory
                path, dirname = os.path.split(path)

                # Skip if this path isn't a directory
                if not os.path.isdir(path):
                    continue

                dirs_to_check.add(path)

        # For each directory which may be empty after cleaning, visit them
        # depth-first and count their descendants
        dir_descendants = dict()
        for path in sorted(dirs_to_check, key=lambda k: -len(k.split(os.path.sep))):
            # Get the absolute path to all the files currently in this directory
            files = [os.path.join(path, f) for f in os.listdir(path)]
            # Filter out the files which we intend to remove
            files = [f for f in files if f not in paths]
            # Compute the minimum number of files potentially contained in this path
            dir_descendants[path] = sum([
                (dir_descendants.get(f, 1) if os.path.isdir(f) else 1)
                for f in files
            ])

            # Schedule the directory for removal if removal of the given files will make it empty
            if dir_descendants[path] == 0:
                paths.append(path)

    # REmove the paths
    for index, path in enumerate(paths):

        # Remove the path
        if os.path.exists(path):
            if os.path.isdir(path):
                logger.out('Removing directory: {}'.format(path))
                if not dry_run:
                    shutil.rmtree(path)
            else:
                logger.out('     Removing file: {}'.format(path))
                if not dry_run:
                    os.remove(path)
        else:
            logger.err('Warning: File {} could not be deleted because it does not exist.'.format(path))

        # Report progress
        event_queue.put(ExecutionEvent(
            'STAGE_PROGRESS',
            job_id=logger.job_id,
            stage_label=logger.stage_label,
            percent=str(index / float(len(paths)))))

    return 0
