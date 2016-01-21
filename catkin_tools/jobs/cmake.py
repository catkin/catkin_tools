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
import subprocess
import sys
import tempfile


from catkin_tools.argument_parsing import handle_make_arguments

from catkin_tools.common import mkdir_p
from catkin_tools.common import get_linked_devel_package_path

from .commands.cmake import CMAKE_EXEC
from .commands.cmake import CMakeIOBufferProtocol
from .commands.cmake import CMakeMakeIOBufferProtocol
from .commands.make import MAKE_EXEC

from .job import get_env_loader
from .job import makedirs

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage
from catkin_tools.execution.stages import FunctionStage

from catkin_tools.terminal_color import ColorMapper

mapper = ColorMapper()
clr = mapper.clr

# FileNotFoundError from Python3
try:
    FileNotFoundError
except NameError:
    class FileNotFoundError(OSError):
        pass


INSTALL_MANIFEST_FILENAME = 'install_manifest.txt'


def get_install_manifest_path(install_target, package_name):
    """Get the path to the installed install manifest for this package."""
    return os.path.join(
        get_linked_devel_package_path(install_target, package_name),
        INSTALL_MANIFEST_FILENAME)


def copy_install_manifest(logger, event_queue, package_name, build_space, install_target):

    # Get the paths
    src_install_manifest_path = os.path.join(build_space, INSTALL_MANIFEST_FILENAME)
    dst_install_manifest_path = get_install_manifest_path(install_target, package_name)

    # Create the directory for the manifest if it doesn't exist
    mkdir_p(os.path.split(dst_install_manifest_path)[0])

    if os.path.exists(src_install_manifest_path):
        # Copy the install manifest
        shutil.copyfile(src_install_manifest_path, dst_install_manifest_path)
    else:
        # Didn't actually install anything, so create an empty manifest for completeness
        logger.err("Warning: No targets installed.")
        with open(dst_install_manifest_path, 'a'):
            os.utime(dst_install_manifest_path, None)

    return 0


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


def get_multiarch():
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


SETUP_FILE_TEMPLATE = """\
#!/usr/bin/env sh
# generated from catkin_tools.jobs.cmake python module

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
export CPATH="{cpath}$CPATH"
export LIBRARY_PATH="{library_path}$LIBRARY_PATH"
export PATH="{path}$PATH"
export PKG_CONFIG_PATH="{pkgcfg_path}$PKG_CONFIG_PATH"
export PYTHONPATH="{pythonpath}$PYTHONPATH"
"""

ENV_FILE_TEMPLATE = """\
#!/usr/bin/env sh
# generated from catkin_tools.jobs.cmake

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  /bin/echo "Calling env.sh without arguments is not supported anymore. Instead\
spawn a subshell and source a setup file manually."
  exit 1
fi

# ensure to not use different shell type which was set before
CATKIN_SHELL=sh

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"
exec "$@"
"""


def generate_env_file(logger, event_queue, context, install_target):
    env_file_path = os.path.join(install_target, 'env.sh')
    if os.path.exists(env_file_path):
        return 0

    env_file_directory = os.path.dirname(env_file_path)
    if not os.path.exists(env_file_directory):
        os.makedirs(env_file_directory)

    logger.out(clr("Generating env file: @!@{yf}{}@|").format(env_file_path))

    # Create a temporary file in the setup_file_directory, so os.rename cannot fail
    tmp_dst_handle, tmp_dst_path = tempfile.mkstemp(
        dir=env_file_directory,
        prefix=os.path.basename(env_file_path) + '.')

    # Write the filled template to the file
    subs = {}
    data = ENV_FILE_TEMPLATE.format(**subs)
    os.write(tmp_dst_handle, data.encode('utf-8'))
    os.close(tmp_dst_handle)

    # Do an atomic rename with os.rename
    os.rename(tmp_dst_path, env_file_path)
    os.chmod(env_file_path, 0o755)

    return 0


def generate_setup_file(logger, event_queue, context, install_target):

    # Create full path to setup file
    setup_file_path = os.path.join(install_target, 'setup.sh')

    # Check if the setup file needs to be generated
    if context.install:
        # Create the setup file in the install space
        setup_file_needed = context.isolate_install or not os.path.exists(setup_file_path)
    else:
        # Do not replace existing setup.sh if devel space is merged
        setup_file_needed = not context.link_devel and context.isolate_devel or not os.path.exists(setup_file_path)

    if not setup_file_needed:
        logger.out("Setup file does not need to be generated.")
        return 0
    else:
        logger.out(clr("Generating setup file: @!@{yf}{}@|").format(setup_file_path))

    # Create the setup file that dependant packages will source
    arch = get_multiarch()
    subs = {}
    subs['cmake_prefix_path'] = install_target + ":"
    subs['ld_path'] = os.path.join(install_target, 'lib') + ":"
    pythonpath = os.path.join(install_target, get_python_install_dir())
    subs['pythonpath'] = pythonpath + ':'
    subs['pkgcfg_path'] = os.path.join(install_target, 'lib', 'pkgconfig') + ":"
    subs['path'] = os.path.join(install_target, 'bin') + ":"
    subs['cpath'] = os.path.join(install_target, 'include') + ":"
    subs['library_path'] = os.path.join(install_target, 'lib') + ":"
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

    # Write the filled template to the file
    data = SETUP_FILE_TEMPLATE.format(**subs)
    os.write(tmp_dst_handle, data.encode('utf-8'))
    os.close(tmp_dst_handle)

    # Do an atomic rename with os.rename
    os.rename(tmp_dst_path, setup_file_path)

    return 0


def create_cmake_build_job(context, package, package_path, dependencies, force_cmake, pre_clean):

    # Package source space path
    pkg_dir = os.path.join(context.source_space_abs, package_path)

    # Package build space path
    build_space = context.package_build_space(package)
    # Package devel space path
    devel_space = context.package_devel_space(package)
    # Package install space path
    install_space = context.package_install_space(package)

    # Get actual staging path
    dest_path = context.package_dest_path(package)
    final_path = context.package_final_path(package)

    # Create job stages
    stages = []

    # Create package build space
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=build_space))

    # CMake command
    makefile_path = os.path.join(build_space, 'Makefile')
    if not os.path.isfile(makefile_path) or force_cmake:
        stages.append(CommandStage(
            'cmake',
            ([CMAKE_EXEC,
              pkg_dir,
              '--no-warn-unused-cli',
              '-DCMAKE_INSTALL_PREFIX=' + final_path] +
             context.cmake_args),
            cwd=build_space,
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir)
        ))
    else:
        stages.append(CommandStage(
            'check',
            [MAKE_EXEC, 'cmake_check_build_system'],
            cwd=build_space,
            logger_factory=CMakeIOBufferProtocol.factory_factory(pkg_dir)
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
    stages.append(CommandStage(
        'make',
        [MAKE_EXEC] + handle_make_arguments(context.make_args),
        cwd=build_space,
        logger_factory=CMakeMakeIOBufferProtocol.factory
    ))

    # Make install command (always run on plain cmake)
    stages.append(CommandStage(
        'install',
        [MAKE_EXEC, 'install'],
        cwd=build_space,
        logger_factory=CMakeMakeIOBufferProtocol.factory,
        locked_resource='installspace'
    ))

    # Copy install manifest
    stages.append(FunctionStage(
        'register',
        copy_install_manifest,
        package_name=package.name,
        build_space=build_space,
        install_target=dest_path))

    # Determine the location where the setup.sh file should be created
    stages.append(FunctionStage(
        'setupgen',
        generate_setup_file,
        context=context,
        install_target=dest_path))

    stages.append(FunctionStage(
        'envgen',
        generate_env_file,
        context=context,
        install_target=dest_path))

    return Job(
        jid=package.name,
        deps=dependencies,
        env_loader=get_env_loader(package, context),
        stages=stages)


def create_cmake_clean_job(context, package_name, dependencies):
    """Factory for a Job to clean cmake packages"""

    # Determine install target
    install_target = context.install_space_abs if context.install else context.devel_space_abs

    # Setup build variables
    build_space = get_package_build_space_path(context.build_space_abs, package_name)

    # Read install manifest
    install_manifest_path = get_install_manifest_path(install_target, package_name)
    installed_files = set()
    if os.path.exists(install_manifest_path):
        with open(install_manifest_path) as f:
            installed_files = set([line.strip() for line in f.readlines()])

    # List of directories to check for removed files
    dirs_to_check = set()

    # Stages for this clean job
    stages = []

    for installed_file in installed_files:
        # Make sure the file is given by an absolute path and it exists
        if not os.path.isabs(installed_file) or not os.path.exists(installed_file):
            continue

        # Add stages to remove the file or directory
        if os.path.isdir(installed_file):
            stages.append(CommandStage(
                'rmdir'
                [CMAKE_EXEC, '-E', 'remove_directory', installed_file],
                cwd=build_space))
        else:
            stages.append(CommandStage(
                'rm',
                [CMAKE_EXEC, '-E', 'remove', installed_file],
                cwd=build_space))

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
        stages.append(CommandStage(
            'rmdir',
            [CMAKE_EXEC, '-E', 'remove_directory', generated_dir],
            cwd=build_space))

    stages.append(CommandStage(
        'rmbuild',
        [CMAKE_EXEC, '-E', 'remove_directory', build_space],
        cwd=context.build_space_abs))

    return Job(
        jid=package_name,
        deps=dependencies,
        stages=stages)


description = dict(
    build_type='cmake',
    description="Builds a plain CMake package.",
    create_build_job=create_cmake_build_job
)
