
from __future__ import print_function

import os
import shutil

from ....utils import in_temporary_directory
from ....utils import assert_cmd_success
from ....utils import assert_cmd_failure
from ....utils import assert_files_exist
from ....utils import catkin_success
from ....utils import catkin_failure
from ....utils import redirected_stdio


from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_no_warnings

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')


@in_temporary_directory
def test_build_pkg_unit_tests():
    """Test running working unit tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests'), source_space)
    with redirected_stdio() as (out, err):
        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose', '--no-deps',
             'python_tests', '--make-args', 'run_tests'])
        assert_cmd_success(['catkin_test_results', 'build/python_tests'])

        assert catkin_success(
            ['run_tests', 'python_tests', '--no-deps', '--no-notify', '--no-status'])
        assert_cmd_success(['catkin_test_results', 'build/python_tests'])


@in_temporary_directory
def test_build_pkg_unit_tests_broken():
    """Test running broken unit tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests_err'), source_space)

    with redirected_stdio() as (out, err):
        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose', '--no-deps',
             'python_tests_err', '--make-args', 'run_tests'])
        assert_cmd_failure(['catkin_test_results', 'build/python_tests_err'])

        assert catkin_success(
            ['run_tests', 'python_tests_err', '--no-deps', '--no-notify', '--no-status'])
        assert_cmd_failure(['catkin_test_results', 'build/python_tests_err'])
