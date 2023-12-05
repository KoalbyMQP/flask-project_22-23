class Motor():
    def __init__(self, mass, M, home):
        self.mass = mass
        self.M = M
        self.home = home
        self.motors = []
class SimLink(Motor):
    def __init__(self, mass, M, home):
        self.mass = mass
        self.M = M
        self.home = home
        self.motors = []

class RealLink(Motor):
    def __init__(self, mass, M, home):
        self.mass = mass
        self.M = M
        self.home = home
        self.motors = []