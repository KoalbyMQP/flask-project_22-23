import matplotlib.pyplot as plt

class Plotter():
    def __init__(self):
        self.fig = plt.figure()
        self.Xs = []
        self.Ys = []
        self.Zs = []
        self.ax = self.fig.add_subplot(1, 1, 1, projection="3d")
        self.plotCounter = 10
        self.plotCalls = 0

    def addPoint(self, point):
        self.plotCalls += 1
        self.Xs.append(point[0])
        self.Ys.append(point[1])
        self.Zs.append(point[2])
        if self.plotCalls == 10:
            self.plotCalls = 0
            self.ax.clear()
            self.ax.plot(self.Xs, self.Ys, self.Zs)
            plt.pause(0.00001)