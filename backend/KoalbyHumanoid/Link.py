class Link():
    def __init__(self, mass, M):
        self.mass = mass
        self.M = M
class SimLink(Link):
    def __init__(self, mass, M):
        self.mass = mass
        self.M = M

class RealLink(Link):
    def __init__(self, mass, M):
        self.mass = mass
        self.M = M
