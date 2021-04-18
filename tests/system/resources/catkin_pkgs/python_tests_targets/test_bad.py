#!/usr/bin/env python

import unittest


class TestBad(unittest.TestCase):

    def test_zero(self):
        self.assertEqual(0, 1)

if __name__ == '__main__':
    unittest.main()
