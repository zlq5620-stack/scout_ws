import unittest,sys,math
from pathlib import Path
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2]/'scout_avoidance')]
from scout_planning.footprint_planner import FootprintPlanner
from scout_planning.grid_planner import GridPlanner

class FootprintPlannerTests(unittest.TestCase):
    def corridor(self,width):
        g=np.full((60,100),100);lo=30-round(width/.05/2);hi=30+round(width/.05/2);g[lo:hi,5:95]=0
        return g
    def test_narrow_straight(self):
        g=self.corridor(.8);p=FootprintPlanner(g,.05,(0,0,0))
        path,state,_=p.search((1.,1.5,0),(3.,1.5,0),time_budget=2)
        self.assertEqual(state,'PATH_READY');self.assertTrue(p.clear(path))
        old=GridPlanner(g,.05,(0,0,0),.425)
        self.assertFalse(old.usable(old.world_to_cell(1,1.5)))
    def test_too_narrow(self):
        p=FootprintPlanner(self.corridor(.6),.05,(0,0,0))
        self.assertEqual(p.search((1,1.5,0),(3,1.5,0))[1],'START_BLOCKED')
    def test_turning_corner_and_goal_orientation(self):
        p=FootprintPlanner(self.corridor(.8),.05,(0,0,0))
        self.assertTrue(p.clear([[2,1.5,0]]))
        self.assertFalse(p.clear([[2,1.5,math.pi/4]]))
        self.assertIsNone(p.segment((2,1.5,0),(2,1.5,math.pi/2)))
        self.assertIsNone(p.connector((2,1.5,0),(2.1,1.5,math.pi/2)))
    def test_rotation_in_room(self):
        g=np.zeros((70,70));p=FootprintPlanner(g,.05,(0,0,0))
        path,state,_=p.search((1,1,0),(1,1,math.pi/2),time_budget=2)
        self.assertEqual(state,'PATH_READY');self.assertAlmostEqual(path[-1][2],math.pi/2)
        self.assertTrue(p.clear(path))
    def test_obstacle_detour(self):
        g=np.zeros((80,100));g[30:45,40:50]=100
        p=FootprintPlanner(g,.05,(0,0,0))
        path,state,n=p.search((1,1.8,0),(3.5,1.8,0),time_budget=2)
        self.assertEqual(state,'PATH_READY');self.assertTrue(p.clear(path))
    def test_unknown_boundary_budget(self):
        g=np.zeros((80,100));g[30,20]=-1;p=FootprintPlanner(g,.05,(0,0,0))
        self.assertFalse(p.clear([[1,1.5,0]]));self.assertFalse(p.clear([[.2,1.,0]]))
        self.assertEqual(p.search((2,2,0),(4,2,0),time_budget=0)[1],'SEARCH_TIME_LIMIT')

if __name__=='__main__':unittest.main()
