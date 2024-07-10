from action import action
from dataclasses import dataclass

@dataclass
class MyPoint:
    x: float
    y: float
    z: float
if __name__ == '__main__':
    action = action.Action()
    action.start()
    action.speak("Hello")
    #action.rotate_head(Direction.LEFT_UP)
    #print("Rotating head: LEFT_UP")
    #action.rotate_body(Direction.LEFT_UP)
    #print("Rotating body: LEFT_UP")
    #action.init_head()
    #action.robot.face_and_move_toward_pt(MyPoint(1, 0, 0))
    #action.robot.face_and_move_toward_pt(MyPoint(1, 1, 0))
    action.register_initial_position()
    # action.return_to_origin()
    print("bring1")
    action.bring2(1, -20)
    print("rtip")
    action.return_to_initial_position()
    #action.return_to_origin()
    action.bring2(0.5, 20)
    # action.rotate_body(Direction.LEFT_UP)
    # print("Rotating body: LEFT_UP")
    # action.rotate_head(Direction.RIGHT_DOWN)
    # print("Rotating head: RIGHT_DOWN")
    # action.rotate_body(Direction.RIGHT_DOWN)
    # print("Rotating body: RIGHT_DOWN")
    rospy.spin()

