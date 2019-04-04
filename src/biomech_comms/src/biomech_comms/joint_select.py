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
        selection = ((area & 3) << 6) | ((joint & 7) << 3) | state & 7
        return selection

    @staticmethod
    def encode_select_to_msg_double(select):
        byte_select = np.array(select, dtype=np.uint64).tobytes()
        float_select = np.frombuffer(byte_select,dtype=np.float64)[0]
        return float_select

