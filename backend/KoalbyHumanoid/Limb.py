class Limb():
    def __init__(self, motors):
        self.motors = motors

class SimLimb(Limb):
    def __init__(self, motors):
        self.motors = motors

class RealLimb(Limb):
    def __init__(self, motors):
        self.motors = motors
