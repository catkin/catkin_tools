
from __future__ import print_function

import os
import shutil

from math import floor

from ...workspace_factory import workspace_factory

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

BUILD = ['build', '--no-color', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--force']  # , '--no-notify', '--no-color', '--no-status']


def test_build_this():
    """Test package context awareness"""
    pass  # TODO: Implement this (both negative and positive results)


def test_start_with_this():
    """Test package context awareness for --start-with option"""
    pass  # TODO: Implement this (both negative and positive results)
