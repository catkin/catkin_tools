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

from catkin_tools.argument_parsing import handle_make_arguments

from catkin_tools.common import mkdir_p

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage
from catkin_tools.execution.stages import FunctionStage

from .commands.cmake import CMAKE_EXEC
from .commands.cmake import CMakeIOBufferProtocol
from .commands.cmake import CMakeMakeIOBufferProtocol
from .commands.make import MAKE_EXEC

from .job import get_env_loader
from .job import makedirs

CTR_NUKE_SH = """
#!/usr/bin/env sh
unset CATKIN_TEST_RESULTS_DIR
unset ROS_TEST_RESULTS_DIR
"""


def ctr_nuke(logger, event_queue, prefix):
    """Adds a shell script which clears the catkin and ros test results dir."""
    ctr_nuke_path = os.path.join(prefix, 'etc', 'catkin', 'profile.d')
    ctr_nuke_filename = os.path.join(ctr_nuke_path, '06-ctr-nuke.sh')
    mkdir_p(ctr_nuke_path)
    if not os.path.exists(ctr_nuke_filename):
        with open(ctr_nuke_filename, 'w') as ctr_nuke_file:
            ctr_nuke_file.write(CTR_NUKE_SH)
    return 0

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

    # Construct CMake command
    makefile_path = os.path.join(build_space, 'Makefile')
    if not os.path.isfile(makefile_path) or force_cmake:
        # Create a shell script which clears the catkin and ros test results environment variable.
        stages.append(FunctionStage(
            'ctr-nuke',
            ctr_nuke,
            prefix=(devel_space if not context.install else install_space)))
        stages.append(CommandStage(
            'cmake',
            ([CMAKE_EXEC,
              pkg_dir,
              '--no-warn-unused-cli',
              '-DCATKIN_DEVEL_PREFIX=' + devel_space,
              '-DCMAKE_INSTALL_PREFIX=' + install_space] +
             context.cmake_args),
            cwd=build_space,
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir),
            occupy_job=True)
        )
    else:
        stages.append(CommandStage(
            'check',
            [MAKE_EXEC, 'cmake_check_build_system'],
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
            [MAKE_EXEC, 'clean'] + make_args,
            cwd=build_space,
        ))

    # Make command
    make_args = handle_make_arguments(
        context.make_args + context.catkin_make_args)
    stages.append(CommandStage(
        'make',
        [MAKE_EXEC] + make_args,
        cwd=build_space,
        env_overrides=ctr_env if 'test' in make_args else {},
        logger_factory=CMakeMakeIOBufferProtocol.factory
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

description = dict(
    build_type='catkin',
    description="Builds a catkin package.",
    create_build_job=create_catkin_build_job
)
