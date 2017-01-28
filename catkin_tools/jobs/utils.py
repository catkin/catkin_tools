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

from catkin_tools.common import mkdir_p
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace

from catkin_tools.resultspace import get_resultspace_environment

from catkin_tools.execution.events import ExecutionEvent


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


def loadenv(logger, event_queue, job_env, package, context):
    # Get the paths to the env loaders
    env_loader_paths = get_env_loaders(package, context)
    # If DESTDIR is set, set _CATKIN_SETUP_DIR as well
    if context.destdir is not None:
        job_env['_CATKIN_SETUP_DIR'] = context.package_dest_path(package)

    for env_loader_path in env_loader_paths:
        if logger:
            logger.out('Loading environment from: {}'.format(env_loader_path))
        resultspace_env = get_resultspace_environment(
            os.path.split(env_loader_path)[0],
            base_env=job_env,
            quiet=True,
            cached=context.use_env_cache,
            strict=False)
        job_env.update(resultspace_env)

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
