
from __future__ import print_function

import os
import shutil
import mock

from catkin_tools import config

from ..utils import assert_raises_regex
from ..utils import in_temporary_directory
from ..utils import redirected_stdio
from ..utils import assert_cmd_success, assert_cmd_failure
from ..utils import assert_files_exist

from ..workspace_assertions import assert_workspace_initialized
from ..workspace_assertions import assert_warning_message
from ..workspace_assertions import assert_no_warnings

@in_temporary_directory
def test_build_no_src():
    out = assert_cmd_failure(['catkin', 'build'])

@in_temporary_directory
def test_build_auto_init_no_pkgs():
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    print("Creating source directory: %s" % source_space)
    os.mkdir(source_space)
    out = assert_cmd_failure(['catkin', 'build'])
    assert_no_warnings(out)
    assert_workspace_initialized('.')

@in_temporary_directory
def test_build_auto_init_one_pkg():
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    print("Creating source directory: %s" % source_space)
    os.mkdir(source_space)
    assert_cmd_success(['catkin', 'create', 'pkg', '--rosdistro', 'hydro', '-p', source_space, 'pkg_a'])
    out = assert_cmd_success(['catkin', 'build', '--no-notify', '--no-status', '--verbose'])
    assert_no_warnings(out)
    assert_workspace_initialized('.')

