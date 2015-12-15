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
import threading


from catkin_tools.argument_parsing import handle_make_arguments

from catkin_tools.common import mkdir_p

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage
from catkin_tools.execution.stages import FunctionStage

from .commands.cmake import CMAKE_EXEC
from .commands.cmake import CMakeIOBufferProtocol
from .commands.make import MAKE_EXEC

from .job import create_env_file
from .job import get_env_file_path
from .job import get_env_loader
from .job import get_package_build_space_path
from .job import makedirs


# job factories

def create_catkin_build_job(context, package, package_path, dependencies, force_cmake, pre_clean):
    """Job class for building catkin packages"""

    # Package source space path
    pkg_dir = os.path.join(context.source_space_abs, package_path)

    # Package build space path
    build_space = context.package_build_space(package)
    # Package devel space path
    devel_space = context.package_devel_space(package)
    # Package install space path
    install_space = context.package_install_space(package)

    # Create job stages
    stages = []

    # Create package build space
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=build_space))

    # Only use the env prefix for building if the develspace is isolated
    env_prefix = []
    if context.isolate_devel:
        # Create an environment file
        env_file_path = get_env_file_path(package, context)
        stages.append(FunctionStage(
            'envgen',
            create_env_file,
            package=package,
            context=context,
            env_file_path=env_file_path))

        env_prefix = [env_file_path]

    # Construct CMake command
    makefile_path = os.path.join(build_space, 'Makefile')
    if not os.path.isfile(makefile_path) or force_cmake:
        stages.append(CommandStage(
            'cmake',
            (env_prefix + [
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

    # Make install command, if installing
    if context.install:
        stages.append(CommandStage(
            'install',
            env_prefix + [MAKE_EXEC, 'install'],
            cwd=build_space))

    return Job(
        jid=package.name,
        deps=dependencies,
        env_loader=get_env_loader(package, context),
        stages=stages)

description = dict(
    build_type='catkin',
    description="Builds a catkin package.",
    create_build_job=create_catkin_build_job
)
