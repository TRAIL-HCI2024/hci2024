from vision import vision
from action import action

if __name__ == '__main__':
    vis = vision.Vision()
    vis.start()
    # audio関連も起動

    # Action
    act = action.Action()
    act.start()
