#!/usr/bin/env python
try:
    from collections import Callable
except ImportError:
    # https://stackoverflow.com/a/70641487
    import collections
    collections.Callable = collections.abc.Callable

import unittest


class TestGood(unittest.TestCase):

    def test_zero(self):
        self.assertEqual(0, 0)

if __name__ == '__main__':
    unittest.main()
