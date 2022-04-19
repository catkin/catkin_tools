import os
import shutil

from ....utils import catkin_failure
from ....utils import catkin_success
from ....utils import in_temporary_directory
from ....utils import redirected_stdio

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
TEST = ['test', '--no-notify', '--no-status']

@in_temporary_directory
def test_catkin_success():
    """Test running working unit tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests'), source_space)
    with redirected_stdio():
        assert catkin_success(BUILD + ['python_tests'])
        assert catkin_success(TEST + ['python_tests'])


@in_temporary_directory
def test_catkin_failure():
    """Test running broken unit tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests_err'), source_space)

    with redirected_stdio():
        assert catkin_success(BUILD + ['python_tests_err'])
        assert catkin_failure(TEST + ['python_tests_err'])


@in_temporary_directory
def test_cmake_success():
    """Test vanilla cmake package"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'test_pkg'), source_space)

    with redirected_stdio():
        assert catkin_success(BUILD + ['test_pkg'])
        assert catkin_success(TEST + ['test_pkg'])


@in_temporary_directory
def test_cmake_failure():
    """Test failing vanilla cmake package"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'test_err_pkg'), source_space)

    with redirected_stdio():
        assert catkin_success(BUILD + ['test_err_pkg'])
        assert catkin_failure(TEST + ['test_err_pkg'])


@in_temporary_directory
def test_skip_missing_test():
    """Test to skip packages without tests"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'cmake_pkgs', 'cmake_pkg'), source_space)

    with redirected_stdio():
        assert catkin_success(BUILD + ['cmake_pkg'])
        assert catkin_success(TEST)


@in_temporary_directory
def test_other_target():
    """Test with a manually specified target"""
    cwd = os.getcwd()
    source_space = os.path.join(cwd, 'src')
    shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'python_tests_targets'), source_space)

    with redirected_stdio():
        assert catkin_success(BUILD + ['python_tests_targets'])
        assert catkin_success(TEST + ['--test-target', 'run_tests_python_tests_targets_nosetests_test_good.py'])
        assert catkin_failure(TEST + ['--test-target', 'run_tests_python_tests_targets_nosetests_test_bad.py'])
