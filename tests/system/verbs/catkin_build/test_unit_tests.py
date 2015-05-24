
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
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    print("Creating source directory: %s" % source_space)
    shutil.copytree(RESOURCES_DIR, source_space)
    with redirected_stdio() as (out, err):
        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose', '--no-deps',
             'pkg_with_test', '--make-args', 'run_tests'])
        assert_cmd_success(['catkin_test_results', 'build/pkg_with_test'])

        assert catkin_success(
            ['build', '--no-notify', '--no-status', '--verbose', '--no-deps',
             'pkg_with_broken_test', '--make-args', 'run_tests'])
        assert_cmd_failure(['catkin_test_results', 'build/pkg_with_broken_test'])


@in_temporary_directory
def test_build_pkg_unit_tests_alias():
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    print("Creating source directory: %s" % source_space)
    shutil.copytree(RESOURCES_DIR, source_space)

    assert catkin_success(['run_tests', 'pkg_with_test', '--no-deps',
                           '--no-notify', '--no-status'])
    assert_cmd_success(['catkin_test_results', 'build/pkg_with_test'])

    assert catkin_success(['run_tests', 'pkg_with_broken_test',
                           '--no-deps', '--no-notify', '--no-status'])
    assert_cmd_failure(['catkin_test_results', 'build/pkg_with_broken_test'])
