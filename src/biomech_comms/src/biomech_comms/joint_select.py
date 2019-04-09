import numpy as np

class JointSelect:
    class AreaSelect:
        LEFT_LEFT = 0
        RIGHT_LEG = 1

    class LegJointSelect:
        ANKLE = 0
        KNEE = 1

    @staticmethod
    def select_joint(area,joint,state):
        selection = (area, joint, state);
        return selection

    @staticmethod
    def encode_select_to_msg_double(select):
        int_select = np.array(select, dtype=np.uint64)
        float_select = int_select.astype(dtype=np.float32)
        return tuple(float_select)

    @staticmethod
    def unselect_joint(selection):
        return selection

    @staticmethod
    def decode_msg_to_joint_select(msg):
        float_select = np.array(msg, dtype=np.float32)
        return tuple(float_select.astype(int))
