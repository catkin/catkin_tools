import os
import shutil

from ....utils import in_temporary_directory
from ....utils import assert_cmd_success
from ....utils import assert_cmd_failure
from ....utils import catkin_success
from ....utils import catkin_failure
from ....utils import redirected_stdio

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')


@in_temporary_directory
def test_catkin_success():
    """Test running working unit tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests'), source_space)
    with redirected_stdio() as (out, err):
        assert catkin_success(['build', 'python_tests', '--no-notify', '--no-status'])
        assert catkin_success(['test', 'python_tests', '--no-notify', '--no-status'])


@in_temporary_directory
def test_catkin_failure():
    """Test running broken unit tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests_err'), source_space)

    with redirected_stdio() as (out, err):
        assert catkin_success(['build', 'python_tests_err', '--no-notify', '--no-status'])
        assert catkin_failure(['test', 'python_tests_err', '--no-notify', '--no-status'])


@in_temporary_directory
def test_cmake_success():
    """Test vanilla cmake package"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'test_pkg'), source_space)

    with redirected_stdio() as (out, err):
        assert catkin_success(['build', 'test_pkg', '--no-notify', '--no-status'])
        assert catkin_success(['test', 'test_pkg', '--no-notify', '--no-status'])


@in_temporary_directory
def test_cmake_failure():
    """Test failing vanilla cmake package"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'test_err_pkg'), source_space)

    with redirected_stdio() as (out, err):
        assert catkin_success(['build', 'test_err_pkg', '--no-notify', '--no-status'])
        assert catkin_failure(['test', 'test_err_pkg', '--no-notify', '--no-status'])


@in_temporary_directory
def test_skip_missing_test():
    """Test to skip packages without tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'cmake_pkg'), source_space)

    with redirected_stdio() as (out, err):
        assert catkin_success(['build', 'cmake_pkg', '--no-notify', '--no-status'])
        assert catkin_success(['test', '--no-notify', '--no-status'])


@in_temporary_directory
def test_other_target():
    """Test with a manually specified target"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests_targets'), source_space)

    with redirected_stdio() as (out, err):
        assert catkin_success(['build', 'python_tests_targets', '--no-notify', '--no-status'])
        assert catkin_success(['test', '--test-target', 'run_tests_python_tests_targets_nosetests_test_good.py',
                               '--no-notify', '--no-status'])
        assert catkin_failure(['test', '--test-target', 'run_tests_python_tests_targets_nosetests_test_bad.py',
                               '--no-notify', '--no-status'])
