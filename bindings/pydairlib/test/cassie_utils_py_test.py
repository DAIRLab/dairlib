import unittest
from cassie_utils import makeFixedBaseCassieTreePointer

class TestCassieUtils(unittest.TestCase):
	def test_makeFixedBaseCassieTreePointer(self):
		tree = makeFixedBaseCassieTreePointer()
		num_q = 16
		num_v = 16
		self.assertEqual(tree.number_of_positions(), num_q)
		self.assertEqual(tree.number_of_positions(), num_v)

if __name__ == '__main__':
    unittest.main()
