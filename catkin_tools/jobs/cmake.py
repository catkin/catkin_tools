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

from .commands.cmake import CMAKE_EXEC
from .commands.cmake import CMAKE_INSTALL_MANIFEST_FILENAME
from .commands.cmake import CMakeIOBufferProtocol
from .commands.cmake import CMakeMakeIOBufferProtocol
from .commands.cmake import CMakeMakeRunTestsIOBufferProtocol
from .commands.cmake import get_installed_files
from .commands.make import MAKE_EXEC

from .utils import copyfiles
from .utils import loadenv
from .utils import makedirs
from .utils import require_command
from .utils import rmfiles

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage
from catkin_tools.execution.stages import FunctionStage

from catkin_tools.terminal_color import ColorMapper

mapper = ColorMapper()
clr = mapper.clr


def copy_install_manifest(
        logger, event_queue,
        src_install_manifest_path,
        dst_install_manifest_path):
    """Copy the install manifest file from one path to another,"""

    # Get file paths
    src_install_manifest_file_path = os.path.join(src_install_manifest_path, CMAKE_INSTALL_MANIFEST_FILENAME)
    dst_install_manifest_file_path = os.path.join(dst_install_manifest_path, CMAKE_INSTALL_MANIFEST_FILENAME)

    # Create the directory for the manifest if it doesn't exist
    mkdir_p(dst_install_manifest_path)

    if os.path.exists(src_install_manifest_file_path):
        # Copy the install manifest
        shutil.copyfile(src_install_manifest_file_path, dst_install_manifest_file_path)
    else:
        # Didn't actually install anything, so create an empty manifest for completeness
        logger.err("Warning: No targets installed.")
        with open(dst_install_manifest_file_path, 'a'):
            os.utime(dst_install_manifest_file_path, None)

    return 0


def get_python_install_dir(context):
    """Returns the same value as the CMake variable PYTHON_INSTALL_DIR

    The PYTHON_INSTALL_DIR variable is normally set from the CMake file:

        catkin/cmake/python.cmake

    :returns: Python install directory for the system Python
    :rtype: str
    """
    cmake_command = [CMAKE_EXEC]
    cmake_command.extend(context.cmake_args)
    script_path = os.path.join(os.path.dirname(__file__), 'cmake', 'python_install_dir.cmake')
    cmake_command.extend(['-P', script_path])
    p = subprocess.Popen(
        cmake_command,
        cwd=os.path.join(os.path.dirname(__file__), 'cmake'),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # only our message (containing the install directory) is written to stderr
    _, out = p.communicate()
    return out.decode().strip()


def get_multiarch():
    """This function returns the suffix for lib directories on supported
    systems or an empty string it uses two step approach to look for multiarch:
    first run gcc -print-multiarch and if failed try to run
    dpkg-architecture."""
    if not sys.platform.lower().startswith('linux'):
        return ''
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

    # Make the file executable without overwriting the permissions for
    # the group and others (copy r flags to x)
    mode = os.stat(tmp_dst_path).st_mode
    mode |= (mode & 0o444) >> 2
    os.chmod(tmp_dst_path, mode)

    # Do an atomic rename with os.rename
    os.rename(tmp_dst_path, env_file_path)

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

    # Create the setup file that dependent packages will source
    arch = get_multiarch()
    subs = {}
    subs['cmake_prefix_path'] = install_target + ":"
    subs['ld_path'] = os.path.join(install_target, 'lib') + ":"
    pythonpath = os.path.join(install_target, get_python_install_dir(context))
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
    # Package metadata path
    metadata_path = context.package_metadata_path(package)
    # Environment dictionary for the job, which will be built
    # up by the executions in the loadenv stage.
    job_env = dict(os.environ)

    # Get actual staging path
    dest_path = context.package_dest_path(package)
    final_path = context.package_final_path(package)

    # Create job stages
    stages = []

    # Load environment for job.
    stages.append(FunctionStage(
        'loadenv',
        loadenv,
        locked_resource='installspace',
        job_env=job_env,
        package=package,
        context=context
    ))

    # Create package build space
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=build_space
    ))

    # Create package metadata dir
    stages.append(FunctionStage(
        'mkdir',
        makedirs,
        path=metadata_path
    ))

    # Copy source manifest
    stages.append(FunctionStage(
        'cache-manifest',
        copyfiles,
        source_paths=[os.path.join(context.source_space_abs, package_path, 'package.xml')],
        dest_path=os.path.join(metadata_path, 'package.xml')
    ))

    require_command('cmake', CMAKE_EXEC)

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

    require_command('make', MAKE_EXEC)

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
        src_install_manifest_path=build_space,
        dst_install_manifest_path=context.package_metadata_path(package)
    ))

    # Determine the location where the setup.sh file should be created
    stages.append(FunctionStage(
        'setupgen',
        generate_setup_file,
        context=context,
        install_target=dest_path
    ))

    stages.append(FunctionStage(
        'envgen',
        generate_env_file,
        context=context,
        install_target=dest_path
    ))

    return Job(
        jid=package.name,
        deps=dependencies,
        env=job_env,
        stages=stages)


def create_cmake_clean_job(
        context,
        package,
        package_path,
        dependencies,
        dry_run,
        clean_build,
        clean_devel,
        clean_install):
    """Generate a Job to clean a cmake package"""

    # Package build space path
    build_space = context.package_build_space(package)
    # Package metadata path
    metadata_path = context.package_metadata_path(package)
    # Environment dictionary for the job, empty for a clean job
    job_env = {}

    stages = []

    if clean_install and context.install:
        installed_files = get_installed_files(context.package_metadata_path(package))
        stages.append(FunctionStage(
            'cleaninstall',
            rmfiles,
            paths=sorted(installed_files),
            remove_empty=True,
            empty_root=context.install_space_abs,
            dry_run=dry_run))

    if clean_devel and not context.install:
        installed_files = get_installed_files(context.package_metadata_path(package))
        stages.append(FunctionStage(
            'cleandevel',
            rmfiles,
            paths=sorted(installed_files),
            remove_empty=True,
            empty_root=context.devel_space_abs,
            dry_run=dry_run))

    if clean_build:
        stages.append(FunctionStage(
            'rmbuild',
            rmfiles,
            paths=[build_space],
            dry_run=dry_run))

    # Remove cached metadata
    if clean_build and clean_devel and clean_install:
        stages.append(FunctionStage(
            'rmmetadata',
            rmfiles,
            paths=[metadata_path],
            dry_run=dry_run))

    return Job(
        jid=package.name,
        deps=dependencies,
        env=job_env,
        stages=stages)


def create_cmake_test_job(
    context,
    package,
    package_path,
):
    """Generate a job to test a cmake package"""
    # Package build space path
    build_space = context.package_build_space(package)
    # Environment dictionary for the job, which will be built
    # up by the executions in the loadenv stage.
    job_env = dict(os.environ)

    # Create job stages
    stages = []

    # Load environment for job
    stages.append(FunctionStage(
        'loadenv',
        loadenv,
        locked_resource=None,
        job_env=job_env,
        package=package,
        context=context,
        verbose=False,
    ))

    # Make command
    stages.append(CommandStage(
        'make',
        [MAKE_EXEC, 'test'],
        cwd=build_space,
        logger_factory=CMakeMakeRunTestsIOBufferProtocol.factory,
    ))

    return Job(
        jid=package.name,
        deps=[],
        env=job_env,
        stages=stages,
    )


description = dict(
    build_type='cmake',
    description="Builds a plain CMake package.",
    create_build_job=create_cmake_build_job,
    create_clean_job=create_cmake_clean_job,
    create_test_job=create_cmake_test_job,
)


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
