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
import stat

from catkin_tools.common import mkdir_p
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace

from catkin_tools.resultspace import get_resultspace_environment

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage

from .commands.cmake import CMAKE_EXEC


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
        if context.link_devel:
            source_path = os.path.join(context.package_final_path(package), 'env.sh')
        else:
            source_path = os.path.join(context.package_dest_path(package), 'env.sh')
        sources = [source_path]

    return sources


def get_env_loader(package, context):
    """This function returns a function object which extends a base environment
    based on a set of environments to load."""

    def load_env(base_env):
        # Copy the base environment to extend
        job_env = dict(base_env)
        # Get the paths to the env loaders
        env_loader_paths = get_env_loaders(package, context)
        # If DESTDIR is set, set _CATKIN_SETUP_DIR as well
        if context.destdir is not None:
            job_env['_CATKIN_SETUP_DIR'] = context.package_dest_path(package)

        for env_loader_path in env_loader_paths:
            # print(' - Loading resultspace env from: {}'.format(env_loader_path))
            resultspace_env = get_resultspace_environment(
                os.path.split(env_loader_path)[0],
                base_env=job_env,
                quiet=True,
                cached=context.use_env_cache,
                strict=False)
            job_env.update(resultspace_env)

        return job_env

    return load_env


def makedirs(logger, event_queue, path):
    """FunctionStage functor that makes a path of directories."""
    mkdir_p(path)
    return 0


def rmfile(logger, event_queue, path):
    """FunctionStage functor that removes a file."""
    if os.path.exists(path):
        os.remove(path)
    return 0


def rmdirs(logger, event_queue, path):
    """FunctionStage functor that removes a directory tree."""
    if os.path.exists(path):
        shutil.rmtree(path)
    return 0


def create_clean_buildspace_job(context, package, dependencies):
    """Create a job to remove a buildspace only."""
    build_space = context.package_build_space(package)
    if not os.path.exists(build_space):
        # No-op
        return Job(jid=package.name, deps=dependencies, stages=[])

    stages = []

    stages.append(FunctionStage('rmbuild', rmdirs, path=build_space))

    return Job(
        jid=package.name,
        deps=dependencies,
        stages=stages)
