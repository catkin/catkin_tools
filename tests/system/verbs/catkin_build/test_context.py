from __future__ import print_function

import os

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--yes']  # , '--no-notify', '--no-color', '--no-status']


def test_build_this():
    """Test package context awareness"""
    pass  # TODO: Implement this (both negative and positive results)


def test_start_with_this():
    """Test package context awareness for --start-with option"""
    pass  # TODO: Implement this (both negative and positive results)
