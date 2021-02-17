import os
import shutil

from ...workspace_factory import workspace_factory

from ....utils import catkin_success
from ....utils import catkin_failure

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--yes']  # , '--no-notify', '--no-color', '--no-status']


def test_cmake_args():
    """Test passing CMake args to packages."""
    with workspace_factory():
        shutil.copytree(os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'cmake_args'), 'src')

        # cmake_args package requires all three vars to be set
        assert catkin_failure(
            BUILD +
            ['cmake_args', '--no-deps'] +
            ['--cmake-args', '-DVAR1=VAL1'])

        assert catkin_failure(
            BUILD +
            ['cmake_args', '--no-deps'] +
            ['--cmake-args', '-DVAR1=VAL1', '-DVAR2=VAL2'])

        assert catkin_success(
            BUILD +
            ['cmake_args', '--no-deps'] +
            ['--cmake-args', '-DVAR1=VAL1', '-DVAR2=VAL2', '-DVAR3=VAL3'])

        assert catkin_success(
            BUILD +
            ['cmake_args', '--no-deps'] +
            ['--cmake-args', '-DVAR1=VAL1', '-DVAR2=VAL2', '-DVAR3=VAL3', '--'])


def test_no_cmake_args():
    """Test building when disabling CMake args in config"""
    pass  # TODO: Add test which sets cmake args with catkin config, then ignores them when building


def test_additional_cmake_args():
    """Test building when using additional CMake args to those config"""
    pass  # TODO: Add test which sets cmake args with catkin config, then adds more when building


def test_make_args():
    """Test passing arguments to the make command"""
    pass  # TODO: Implement this


def test_additional_make_args():
    """Test building when using additional make args to those config"""
    pass  # TODO: Add test which sets make args with catkin config, then adds more when building


def test_no_make_args():
    """Test building when disabling make args in config"""
    pass  # TODO: Add test which sets make args with catkin config, then ignores them when building


def test_catkin_make_args():
    """Test passing arguments to the make command for catkin packages only"""
    pass  # TODO: Implement this


def test_additional_catkin_make_args():
    """Test building when using additional catkin make args to those config"""
    pass  # TODO: Add test which sets catkin make args with catkin config, then adds more when building


def test_no_catkin_make_args():
    """Test building when disabling catkin make args in config"""
    pass  # TODO: Add test which sets catkin make args with catkin config, then ignores them when building


def test_jobs_args():
    """Test parallel jobs and parallel packages args"""
    pass  # TODO: Run catkin build, then check JobServer singleton
