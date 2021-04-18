#!/usr/bin/env python

import unittest


class TestGood(unittest.TestCase):

    def test_zero(self):
        self.assertEqual(0, 0)

if __name__ == '__main__':
    unittest.main()
