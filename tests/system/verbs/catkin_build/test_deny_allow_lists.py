from __future__ import print_function

import os

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--all', '--yes']  # , '--no-notify', '--no-color', '--no-status']


def test_allowlist():
    """Test building allowlisted packages only"""
    pass  # TODO: Write test


def test_denylist():
    """Test building all packages except denylisted packages"""
    pass  # TODO: Write test


def test_denylist_allowlist():
    """Test building with non-empty denylist and allowlist"""
    pass  # TODO: Write test
