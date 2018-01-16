import os
import sys
import math
import unittest
import numpy as np

ramp_root = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(ramp_root) # directory_name

from ramp_rlpara.rl_exercise.envs.path_dcoe_discre import PathDcoeDiscre

path0 = np.array([[3,2],
                  [5,3],
                  [5,3]])

path1 = np.array([[5,2],
                  [8,3],
                  [10,10]])

path2 = np.array([[7,1],
                  [9,1],
                  [10,10]])

p1len = math.sqrt(25+4) + math.sqrt(9+1) + math.sqrt(4+49)
p2len = math.sqrt(49+1) + math.sqrt(4+0) + math.sqrt(1+81)

class TestPathDcoeDiscre(unittest.TestCase):
    """Test case for class PathDcoeDiscre
    """
    def test_setState(self):
        """Test the method setState(D) of class PathDcoeDiscre
        """
        env = PathDcoeDiscre()

        env.setState(0.9)
        self.assertEqual(0.9, env.state)

        env.setState(0.0)
        self.assertEqual(0.0, env.state)

        env.setState(0)
        self.assertEqual(0.0, env.state)

        env.setState(1.0)
        self.assertEqual(1.0, env.state)

        env.setState(1)
        self.assertEqual(1.0, env.state)

        env.setState(1.9)
        self.assertEqual(1.0, env.state)

        env.setState(-0.9)
        self.assertEqual(0.0, env.state)

        env.setState(99999999999999999999999.9)
        self.assertEqual(1.0, env.state)

        env.setState(-99999999999999999999999.9)
        self.assertEqual(0.0, env.state)



    def test_getOb_getLen(self):
        """Test the methods getOb() and getLen() of class PathDcoeDiscre
        """
        env = PathDcoeDiscre()

        env.setState(0.9)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path2 == ob).all())
        self.assertEqual(p2len, plen)

        env.setState(0.0)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path0 == ob).all())
        self.assertEqual(env.max_len, plen)

        env.setState(0)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path0 == ob).all())
        self.assertEqual(env.max_len, plen)

        env.setState(1.0)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path2 == ob).all())
        self.assertEqual(p2len, plen)

        env.setState(1)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path2 == ob).all())
        self.assertEqual(p2len, plen)

        env.setState(1.9)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path2 == ob).all())
        self.assertEqual(p2len, plen)

        env.setState(-0.9)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path0 == ob).all())
        self.assertEqual(env.max_len, plen)

        env.setState(99999999999999999999999.9)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path2 == ob).all())
        self.assertEqual(p2len, plen)

        env.setState(-99999999999999999999999.9)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path0 == ob).all())
        self.assertEqual(env.max_len, plen)

        env.setState(0.2)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path1 == ob).all())
        self.assertEqual(p1len, plen)

        env.setState(0.5)
        ob = env.getOb()
        plen = env.getLen(ob)
        self.assertTrue((path2 == ob).all())
        self.assertEqual(p2len, plen)



    def test_getReward(self):
        """Test the getReward() method of PathDcoeDiscre class
        """
        env = PathDcoeDiscre()

        p0r = 0.0
        p1r = env.max_len - p1len
        p2r = env.max_len - p2len

        self.assertEqual(p0r, env.getReward(path0))
        self.assertEqual(p1r, env.getReward(path1))
        self.assertEqual(p2r, env.getReward(path2))

        print(env.getReward(path0))
        print(env.getReward(path1))
        print(env.getReward(path2))



    def test_reset(self):
        """Test the reset() method of PathDcoeDiscre class
        """
        env = PathDcoeDiscre()

        ob = env.reset(0.9)
        self.assertEqual(0.9, env.state)
        self.assertTrue((path2 == ob).all())

        ob = env.reset(0.0)
        self.assertEqual(0.0, env.state)
        self.assertTrue((path0 == ob).all())

        ob = env.reset(0)
        self.assertEqual(0.0, env.state)
        self.assertTrue((path0 == ob).all())

        ob = env.reset(1.0)
        self.assertEqual(1.0, env.state)
        self.assertTrue((path2 == ob).all())

        ob = env.reset(1)
        self.assertEqual(1.0, env.state)
        self.assertTrue((path2 == ob).all())

        ob = env.reset(1.9)
        self.assertEqual(1.0, env.state)
        self.assertTrue((path2 == ob).all())

        ob = env.reset(-0.9)
        self.assertEqual(0.0, env.state)
        self.assertTrue((path0 == ob).all())

        ob = env.reset(99999999999999999999999.9)
        self.assertEqual(1.0, env.state)
        self.assertTrue((path2 == ob).all())

        ob = env.reset(-99999999999999999999999.9)
        self.assertEqual(0.0, env.state)
        self.assertTrue((path0 == ob).all())

        ob = env.reset(0.2)
        self.assertEqual(0.2, env.state)
        self.assertTrue((path1 == ob).all())

        ob = env.reset(0.5)
        self.assertEqual(0.5, env.state)
        self.assertTrue((path2 == ob).all())



    # def test_step(self):
    #     """Test the step() method of PathDcoeDiscre class
    #     """
    #     env = PathDcoeDiscre()
    #     p0r = 0.0
    #     p1r = env.max_len - p1len
    #     p2r = env.max_len - p2len

    #     env.reset()

    #     state0 = env.state
    #     ob, r, d, info = env.step(2)
    #     state1 = state0 + 0.1
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((env.getOb() == ob).all())
    #     self.assertEqual(env.getReward(ob), r)
    #     self.assertEqual(env.done(ob), d)
    #     self.assertEqual({}, info)

    #     state0 = env.state
    #     ob, r, d, info = env.step(1)
    #     state1 = state0
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((env.getOb() == ob).all())
    #     self.assertEqual(env.getReward(ob), r)
    #     self.assertEqual(env.done(ob), d)
    #     self.assertEqual({}, info)

    #     state0 = env.state
    #     ob, r, d, info = env.step(0)
    #     state1 = state0 - 0.1
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((env.getOb() == ob).all())
    #     self.assertEqual(env.getReward(ob), r)
    #     self.assertEqual(env.done(ob), d)
    #     self.assertEqual({}, info)

    #     env.reset(0.4)

    #     ob, r, d, info = env.step(2)
    #     state1 = 0.5
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((path2 == ob).all())
    #     self.assertEqual(p2r, r)
    #     self.assertEqual(True, d)
    #     self.assertEqual({}, info)

    #     env.reset(-0.1)

    #     ob, r, d, info = env.step(0)
    #     state1 = 0.0
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((path0 == ob).all())
    #     self.assertEqual(0.0, r)
    #     self.assertEqual(False, d)
    #     self.assertEqual({}, info)

    #     env.reset(-0.1)

    #     ob, r, d, info = env.step(2)
    #     state1 = 0.1
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((path0 == ob).all())
    #     self.assertEqual(0.0, r)
    #     self.assertEqual(False, d)
    #     self.assertEqual({}, info)

    #     ob, r, d, info = env.step(2)
    #     state1 = 0.2
    #     self.assertEqual(state1, env.state)
    #     self.assertTrue((path1 == ob).all())
    #     self.assertEqual(p1r, r)
    #     self.assertEqual(True, d)
    #     self.assertEqual({}, info)



if __name__ == '__main__':
    unittest.main()