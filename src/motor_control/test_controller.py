#!/usr/bin/env python

import unittest

from scripts.utils import scale_velocity

class MotorControlTestCase(unittest.TestCase):

	def runTest(self):
		w_1 = 0.0
		res_1 = scale_velocity(w_1)
		self.assertEqual(res_1, 0)

		w_2 = 20.0
		res_2 = scale_velocity(w_2)
		self.assertEqual(res_2, 100)

		w_3 = 10.0
		res_3 = scale_velocity(w_3)
		self.assertEqual(res_3, 62.5)

		w_4 = -10.0
		res_4 = scale_velocity(w_4)
		self.assertEqual(res_4, 62.5)

		w_5 = -25.0
		res_5 = scale_velocity(w_5)
		self.assertEqual(res_5, 100)

		w_6 = 25
		res_6 = scale_velocity(w_6)
		self.assertEqual(res_5, 100)

		pass


if __name__ == '__main__':
	unittest.main()