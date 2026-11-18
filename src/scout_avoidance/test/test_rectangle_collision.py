"""Run with python3 -m unittest discover -s src/scout_avoidance/test."""
import math
import sys
import unittest
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scout_avoidance.local_planner_core import Settings, MapCollision, scan_rectangle_clearance, check_motion


class RectangleCollisionTests(unittest.TestCase):
    def test_dimensions(self):
        np.testing.assert_allclose(Settings().half_extents, [.345, .32])

    def test_scan_sides_and_rotating_corner(self):
        # Side point inside old circle, but outside actual padded rectangle.
        self.assertGreater(scan_rectangle_clearance([[0,0,0]], cKDTree([[0,.4]]), .345,.32), 0)
        # Corner outside a long-side-diameter circle must still collide.
        self.assertLess(scan_rectangle_clearance([[0,0,0]], cKDTree([[.34,.31]]), .345,.32), 0)
        tree = cKDTree([[.44,0]])
        self.assertGreater(scan_rectangle_clearance([[0,0,0]], tree,.345,.32), 0)
        self.assertLess(scan_rectangle_clearance([[0,0,math.pi/4]],tree,.345,.32), 0)

    def test_cell_square_and_unknown(self):
        for value in (100,-1):
            grid=np.zeros((100,100));grid[50,53]=value
            m=MapCollision(grid,.1,(-5,-5,0))
            # Obstacle cell starts at x=.3: overlap even though center is outside.
            self.assertFalse(m.clear_rectangle([[0,0,0]],.345,.32))
            self.assertTrue(m.clear_rectangle([[-.1,0,0]],.345,.32))

    def test_rotated_map_and_boundary(self):
        grid=np.zeros((100,100));grid[50,53]=100
        m=MapCollision(grid,.1,(5,-5,math.pi/2))
        self.assertFalse(m.clear_rectangle([[0,0,math.pi/2]],.345,.32))
        self.assertTrue(m.clear_rectangle([[0,-.1,math.pi/2]],.345,.32))
        self.assertFalse(m.clear_rectangle([[4.8,-4.8,0]],.345,.32))

    def test_narrow_corridor_and_turn(self):
        grid=np.zeros((100,100));grid[:46,:]=100;grid[54:,:]=100
        m=MapCollision(grid,.1,(-5,-5,0)) # corridor y=(-.4,.4)
        self.assertTrue(m.clear_rectangle([[0,0,0]],.345,.32))
        self.assertFalse(m.clear_rectangle([[0,0,math.pi/4]],.345,.32))
        context={'map':m,'pose':(0,0,0),'obstacle_tree':cKDTree([[0,.4],[0,-.4]])}
        self.assertIsNotNone(check_motion(.02,0,context,Settings())[0])
        context['obstacle_tree']=cKDTree([[.34,.31]])
        self.assertEqual(check_motion(0,0,context,Settings())[1],'SCAN_COLLISION')


if __name__=='__main__':unittest.main()
