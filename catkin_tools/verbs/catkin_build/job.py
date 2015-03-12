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
import subprocess
import sys
import tempfile

from catkin_tools.argument_parsing import handle_make_arguments
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace
from catkin_tools.utils import which

from .common import create_build_space
from .common import generate_env_file
from .common import get_python_install_dir

MAKE_EXEC = which('make')
CMAKE_EXEC = which('cmake')


class Command(object):

    """Single command which is part of a job"""
    lock_install_space = False
    stage_name = ''

    def __init__(self, env_loader, cmd, location):
        self.cmd = [env_loader] + cmd
        self.cmd_str = ' '.join(self.cmd)
        self.executable = os.path.basename(cmd[0])
        self.pretty = ' '.join([self.executable] + cmd[1:])
        self.plain_cmd = cmd
        self.plain_cmd_str = ' '.join(self.plain_cmd)
        self.env_loader = env_loader
        self.location = location


class MakeCommand(Command):
    stage_name = 'make'

    def __init__(self, env_loader, cmd, location):
        super(MakeCommand, self).__init__(env_loader, cmd, location)

        if MAKE_EXEC is None:
            raise RuntimeError("Executable 'make' could not be found in PATH.")


class CMakeCommand(Command):
    stage_name = 'cmake'

    def __init__(self, env_loader, cmd, location):
        super(CMakeCommand, self).__init__(env_loader, cmd, location)

        if CMAKE_EXEC is None:
            raise RuntimeError("Executable 'cmake' could not be found in PATH.")


class InstallCommand(MakeCommand):

    """Command which touches the install space"""
    lock_install_space = True
    stage_name = 'make install'

    def __init__(self, env_loader, cmd, location):
        super(InstallCommand, self).__init__(env_loader, cmd, location)


class Job(object):

    """Encapsulates a job which builds a package"""

    def __init__(self, package, package_path, context, force_cmake):
        self.package = package
        self.package_path = package_path
        self.context = context
        self.force_cmake = force_cmake
        self.commands = []
        self.__command_index = 0

    def get_commands(self):
        raise NotImplementedError('get_commands')

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):
        if self.__command_index >= len(self.commands):
            raise StopIteration()
        self.__command_index += 1
        return self.commands[self.__command_index - 1]


def create_env_file(package, context):
    sources = []
    source_snippet = '. "{source_path}"'
    # If installing to isolated folders or not installing, but devel spaces are not merged
    if (context.install and context.isolate_install) or (not context.install and context.isolate_devel):
        # Source each package's install or devel space
        space = context.install_space_abs if context.install else context.devel_space_abs
        # Get the recursive dependcies
        depends = get_cached_recursive_build_depends_in_workspace(package, context.packages)
        # For each dep add a line to source its setup file
        for dep_pth, dep in depends:
            source_path = os.path.join(space, dep.name, 'setup.sh')
            sources.append(source_snippet.format(source_path=source_path))
    else:
        # Just source common install or devel space
        source_path = os.path.join(
            context.install_space_abs if context.install else context.devel_space_abs,
            'setup.sh')
        sources = [source_snippet.format(source_path=source_path)] if os.path.exists(source_path) else []
    # Build the env_file
    env_file_path = os.path.abspath(os.path.join(context.build_space_abs, package.name, 'build_env.sh'))
    generate_env_file(sources, env_file_path)
    return env_file_path


# TODO: Move various Job types out to another file

class CMakeJob(Job):

    """Job class for building plain cmake packages"""

    def __init__(self, package, package_path, context, force_cmake):
        Job.__init__(self, package, package_path, context, force_cmake)
        self.commands = self.get_commands()

    def get_multiarch(self):
        if not sys.platform.lower().startswith('linux'):
            return ''
        # this function returns the suffix for lib directories on supported systems or an empty string
        # it uses two step approach to look for multiarch: first run gcc -print-multiarch and if
        # failed try to run dpkg-architecture
        error_thrown = False
        try:
            p = subprocess.Popen(
                ['gcc', '-print-multiarch'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = p.communicate()
        except (OSError, FileNotFoundError):
            error_thrown = True
        if error_thrown or p.returncode != 0:
            try:
                out, err = subprocess.Popen(
                    ['dpkg-architecture', '-qDEB_HOST_MULTIARCH'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()
            except (OSError, FileNotFoundError):
                return ''
        # be sure to return empty string or a valid multiarch tuple
        decoded = out.decode().strip()
        assert(not decoded or decoded.count('-') == 2)
        return decoded

    def get_commands(self):
        commands = []
        # Setup build variables
        pkg_dir = os.path.join(self.context.source_space_abs, self.package_path)
        build_space = create_build_space(self.context.build_space_abs, self.package.name)
        if self.context.isolate_devel:
            devel_space = os.path.join(self.context.devel_space_abs, self.package.name)
        else:
            devel_space = self.context.devel_space_abs
        if self.context.isolate_install:
            install_space = os.path.join(self.context.install_space_abs, self.package.name)
        else:
            install_space = self.context.install_space_abs
        install_target = install_space if self.context.install else devel_space
        # Create an environment file
        env_cmd = create_env_file(self.package, self.context)
        # CMake command
        makefile_path = os.path.join(build_space, 'Makefile')
        if not os.path.isfile(makefile_path) or self.force_cmake:
            commands.append(CMakeCommand(
                env_cmd,
                [
                    CMAKE_EXEC,
                    pkg_dir,
                    '-DCMAKE_INSTALL_PREFIX=' + install_target
                ] + self.context.cmake_args,
                build_space
            ))
            commands[-1].cmd.extend(self.context.cmake_args)
        else:
            commands.append(MakeCommand(env_cmd, [MAKE_EXEC, 'cmake_check_build_system'], build_space))
        # Make command
        commands.append(MakeCommand(
            env_cmd,
            [MAKE_EXEC] + handle_make_arguments(self.context.make_args),
            build_space
        ))
        # Make install command (always run on plain cmake)
        commands.append(InstallCommand(env_cmd, [MAKE_EXEC, 'install'], build_space))
        # Determine the location of where the setup.sh file should be created
        if self.context.install:
            setup_file_path = os.path.join(install_space, 'setup.sh')
            if not self.context.isolate_install and os.path.exists(setup_file_path):
                return commands
        else:  # Create it in the devel space
            setup_file_path = os.path.join(devel_space, 'setup.sh')
            if not self.context.isolate_devel and os.path.exists(setup_file_path):
                # Do not replace existing setup.sh if devel space is merged
                return commands
        # Create the setup file other packages will source when depending on this package
        arch = self.get_multiarch()
        subs = {}
        subs['cmake_prefix_path'] = install_target + ":"
        subs['ld_path'] = os.path.join(install_target, 'lib') + ":"
        pythonpath = os.path.join(install_target, get_python_install_dir())
        subs['pythonpath'] = pythonpath + ':'
        subs['pkgcfg_path'] = os.path.join(install_target, 'lib', 'pkgconfig') + ":"
        subs['path'] = os.path.join(install_target, 'bin') + ":"
        if arch:
            subs['ld_path'] += os.path.join(install_target, 'lib', arch) + ":"
            subs['pkgcfg_path'] += os.path.join(install_target, 'lib', arch, 'pkgconfig') + ":"
        setup_file_directory = os.path.dirname(setup_file_path)
        if not os.path.exists(setup_file_directory):
            os.makedirs(setup_file_directory)
        # Create a temporary file in the setup_file_directory, so os.rename cannot fail
        tmp_dst_handle, tmp_dst_path = tempfile.mkstemp(
            dir=setup_file_directory,
            prefix=os.path.basename(setup_file_path) + '.')
        # Write the fulfilled template to the file
        data = """\
#!/usr/bin/env sh
# generated from catkin_tools.verbs.catkin_build.job python module

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi

# detect if running on Darwin platform
_UNAME=`uname -s`
IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  IS_DARWIN=1
fi

# Prepend to the environment
export CMAKE_PREFIX_PATH="{cmake_prefix_path}$CMAKE_PREFIX_PATH"
if [ $IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH="{ld_path}$LD_LIBRARY_PATH"
else
  export DYLD_LIBRARY_PATH="{ld_path}$DYLD_LIBRARY_PATH"
fi
export PATH="{path}$PATH"
export PKG_CONFIG_PATH="{pkgcfg_path}$PKG_CONFIG_PATH"
export PYTHONPATH="{pythonpath}$PYTHONPATH"
""".format(**subs)
        os.write(tmp_dst_handle, data.encode('utf-8'))
        os.close(tmp_dst_handle)
        # Do an atomic rename with os.rename
        os.rename(tmp_dst_path, setup_file_path)
        return commands


class CatkinJob(Job):

    """Job class for building catkin packages"""

    def __init__(self, package, package_path, context, force_cmake):
        Job.__init__(self, package, package_path, context, force_cmake)
        self.commands = self.get_commands()

    def get_commands(self):
        commands = []
        # Setup build variables
        pkg_dir = os.path.join(self.context.source_space_abs, self.package_path)
        build_space = create_build_space(self.context.build_space_abs, self.package.name)
        if self.context.isolate_devel:
            devel_space = os.path.join(self.context.devel_space_abs, self.package.name)
        else:
            devel_space = self.context.devel_space_abs
        if self.context.isolate_install:
            install_space = os.path.join(self.context.install_space_abs, self.package.name)
        else:
            install_space = self.context.install_space_abs
        # Create an environment file
        env_cmd = create_env_file(self.package, self.context)
        # CMake command
        makefile_path = os.path.join(build_space, 'Makefile')
        if not os.path.isfile(makefile_path) or self.force_cmake:
            commands.append(CMakeCommand(
                env_cmd,
                [
                    CMAKE_EXEC,
                    pkg_dir,
                    '-DCATKIN_DEVEL_PREFIX=' + devel_space,
                    '-DCMAKE_INSTALL_PREFIX=' + install_space
                ] + self.context.cmake_args,
                build_space
            ))
        else:
            commands.append(MakeCommand(env_cmd, [MAKE_EXEC, 'cmake_check_build_system'], build_space))
        # Make command
        commands.append(MakeCommand(
            env_cmd,
            [MAKE_EXEC] + handle_make_arguments(self.context.make_args + self.context.catkin_make_args),
            build_space
        ))
        # Make install command, if installing
        if self.context.install:
            commands.append(InstallCommand(env_cmd, [MAKE_EXEC, 'install'], build_space))
        return commands
