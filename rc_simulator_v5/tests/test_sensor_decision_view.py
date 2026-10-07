import unittest
from types import SimpleNamespace
from config import VehicleConfig
from vehicle_view import VehicleView

class Canvas:
    def __init__(self):self.lines=[];self.labels=[]
    def create_line(self,*args,**kw):self.lines.append((args,kw))
    def create_text(self,*args,**kw):self.labels.append(kw['text'])

class DecisionViewTests(unittest.TestCase):
    def draw(self,state,handle):
        cv=Canvas();view=VehicleView.__new__(VehicleView);view.app=SimpleNamespace(family='TkDefaultFont')
        sim=SimpleNamespace(state=state,program_stopped=False,collision_id=None,
            car=SimpleNamespace(handle=handle),config=VehicleConfig())
        view.draw_decision(cv,sim,lambda p:p)
        return cv

    def test_left_target_and_right_command_are_distinct(self):
        cv=self.draw('ギャップ | target_deg=20.0',-25)
        target=next(args for args,kw in cv.lines if kw['fill']=='#67ff91')
        command=next(args for args,kw in cv.lines if kw['fill']=='white')
        self.assertGreater(target[-1],0);self.assertLess(command[-1],0)

    def test_stop_does_not_show_motion_arrow(self):
        cv=self.draw('安全停止：接近限界 | target_deg=20.0',50)
        self.assertEqual(cv.lines,[])
        self.assertTrue(any('停止判断' in s for s in cv.labels))

    def test_legacy_program_has_no_invented_gap_target(self):
        cv=self.draw('他のプログラム',0)
        self.assertFalse(any(kw['fill']=='#67ff91' for _,kw in cv.lines))
        self.assertTrue(any(kw['fill']=='white' for _,kw in cv.lines))
