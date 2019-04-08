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
        selection = ((area & 255) << 16) | ((joint & 255) << 8) | state & 255;
        return selection

    @staticmethod
    def encode_select_to_msg_double(select):
        byte_select = np.array(select, dtype=np.uint64).tobytes()
        float_select = np.frombuffer(byte_select,dtype=np.float64)[0]
        return float_select

    @staticmethod
    def unselect_joint(selection):
        return ((selection >> 16) & 255, (selection >> 8) & 255, selection & 255)

    @staticmethod
    def decode_msg_to_joint_select(msg):
        byte_select = np.array(msg, dtype=np.float32).tobytes()
        print(len(byte_select))
        int_select = np.frombuffer(byte_select,dtype=np.uint32)[0]
        return int_select
