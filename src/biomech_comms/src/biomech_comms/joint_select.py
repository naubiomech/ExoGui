class JointSelect:
    class AreaSelect:
        LEGS = 0

    class LegSelect:
        LEFT_LEFT = 0
        RIGHT_LEG = 1

    class LegJointSelect:
        ANKLE = 0
        KNEE = 1

    @staticmethod
    def select_joint(area,limb,joint):
        return ((area & 3) << 6) | ((limb & 7) << 3) | joint & 7
