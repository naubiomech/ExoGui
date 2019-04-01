class JointSelect:
    class AreaSelect:
        LEFT_LEFT = 0
        RIGHT_LEG = 1

    class LegJointSelect:
        ANKLE = 0
        KNEE = 1

    @staticmethod
    def select_joint(area,joint,state):
        return ((area & 3) << 6) | ((joint & 7) << 3) | state & 7
