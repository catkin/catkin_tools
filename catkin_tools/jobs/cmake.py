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
import stat
import subprocess
import sys
import tempfile

from multiprocessing import cpu_count

from catkin_tools.argument_parsing import handle_make_arguments

from catkin_tools.runner import run_command

from catkin_tools.utils import which

from .commands.cmake import CMakeCommand
from .commands.cmake import CMAKE_EXEC
from .commands.make import MakeCommand
from .commands.make import InstallCommand
from .commands.make import MAKE_EXEC

from .job import create_build_space
from .job import create_env_file
from .job import BuildJob
from .job import CleanJob

# FileNotFoundError from Python3
try:
    FileNotFoundError
except NameError:
    class FileNotFoundError(OSError):
        pass


INSTALL_MANIFEST_FILE = 'install_manifest.txt'


def get_python_install_dir():
    """Returns the same value as the CMake variable PYTHON_INSTALL_DIR

    The PYTHON_INSTALL_DIR variable is normally set from the CMake file:

        catkin/cmake/python.cmake

    :returns: Python install directory for the system Python
    :rtype: str
    """
    python_install_dir = 'lib'
    if os.name != 'nt':
        python_version_xdoty = str(sys.version_info[0]) + '.' + str(sys.version_info[1])
        python_install_dir = os.path.join(python_install_dir, 'python' + python_version_xdoty)

    python_use_debian_layout = os.path.exists('/etc/debian_version')
    python_packages_dir = 'dist-packages' if python_use_debian_layout else 'site-packages'
    python_install_dir = os.path.join(python_install_dir, python_packages_dir)
    return python_install_dir


class CMakeBuildJob(BuildJob):

    """Job class for building plain cmake packages"""

    def __init__(self, context, package, package_path, force_cmake):
        super(CMakeBuildJob, self).__init__(context, package, package_path, force_cmake)
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


class CMakeCleanJob(CleanJob):

    """Job class for cleaning plain cmake packages"""

    def __init__(self, context, package_name):
        super(CMakeCleanJob, self).__init__(context, package_name)
        self.commands = self.get_commands()

    def get_commands(self):
        commands = []
        # Setup build variables
        build_space = create_build_space(self.context.build_space_abs, self.package_name)

        # Read install manifest
        install_manifest_path = os.path.join(build_space, INSTALL_MANIFEST_FILE)
        installed_files = set()
        if os.path.exists(install_manifest_path):
            with open(install_manifest_path) as f:
                installed_files = set([line.strip() for line in f.readlines()])

        dirs_to_check = set()

        for installed_file in installed_files:
            # Make sure the file is given by an absolute path and it exists
            if not os.path.isabs(installed_file) or not os.path.exists(installed_file):
                continue

            # Add commands to remove the file or directory
            if os.path.isdir(installed_file):
                commands.append(CMakeCommand(
                    None,
                    [CMAKE_EXEC, '-E', 'remove_directory', installed_file],
                    build_space))
            else:
                commands.append(CMakeCommand(
                    None,
                    [CMAKE_EXEC, '-E', 'remove', installed_file],
                    build_space))

            # Check if directories that contain this file will be empty once it's removed
            path = installed_file
            # Only look in the devel space
            while path != self.context.devel_space_abs:
                # Pop up a directory
                path, dirname = os.path.split(path)

                # Skip if this path isn't a directory
                if not os.path.isdir(path):
                    continue

                dirs_to_check.add(path)

        # For each directory which may be empty after cleaning, visit them depth-first and count their descendants
        dir_descendants = dict()
        dirs_to_remove = set()
        for path in sorted(dirs_to_check, key=lambda k: -len(k.split(os.path.sep))):
            # Get the absolute path to all the files currently in this directory
            files = [os.path.join(path, f) for f in os.listdir(path)]
            # Filter out the files which we intend to remove
            files = [f for f in files if f not in installed_files]
            # Compute the minimum number of files potentially contained in this path
            dir_descendants[path] = sum([(dir_descendants.get(f, 1) if os.path.isdir(f) else 1) for f in files])

            # Schedule the directory for removal if removal of the given files will make it empty
            if dir_descendants[path] == 0:
                dirs_to_remove.add(path)

        for generated_dir in dirs_to_remove:
            commands.append(CMakeCommand(
                None,
                [CMAKE_EXEC, '-E', 'remove_directory', generated_dir],
                build_space))

        return commands
