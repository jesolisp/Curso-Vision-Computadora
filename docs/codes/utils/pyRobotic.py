import time
import glob
import numpy as np
import pyvista as pv
from threading import Thread
from pyvistaqt import BackgroundPlotter


class robotics:
    def __init__(self, path="", color=None):
        self.color = color
        self.path = path
        filenames = glob.glob(self.path + "/*.stl")
        self.robot = []
        self.robotCopy = []
        self.isTrajectory = False

        # Load the STL files and create meshes
        for filename in filenames:
            self.robot.append(pv.read(filename))
            self.robotCopy.append(pv.read(filename))

    def configureScene(self, bounds, window_size=(1024, 768), title="Omnidirectional robot"):
        self.bounds = bounds
        self.plotter = BackgroundPlotter(window_size=window_size, title=title)
        self.plotter.set_background('white')

    def initRobot(self, x1, y1, phi, escala):
        self.x1 = x1
        self.y1 = y1
        self.phi = phi
        self.escala = escala

        # Scale and color robot meshes
        for i in range(len(self.robot)):
            self.robot[i].points *= self.escala
            self.robotCopy[i].points *= self.escala
            if self.color is None:
                self.plotter.add_mesh(self.robotCopy[i], color='black')
            else:
                self.plotter.add_mesh(self.robotCopy[i], color=self.color[i])

    def initTrajectory(self, hx, hy):
        self.isTrajectory = True
        self.hx = hx
        self.hy = hy
        self.sizeh = len(self.hx)
        points = np.column_stack((np.zeros(self.sizeh), np.zeros(self.sizeh), np.zeros(self.sizeh)))
        self.spline = pv.Spline(points, self.sizeh)
        self.plotter.add_mesh(self.spline, color='red', line_width=4)

    def plotDesiredTrajectory(self, hxd, hyd):
        sizehd = len(hxd)
        points = np.column_stack((hxd, hyd, np.zeros(sizehd)))
        self.spline1 = pv.Spline(points, sizehd)
        self.plotter.add_mesh(self.spline1, color='blue', line_width=4)

    def startSimulation(self, step=1, ts=0.1):
        self.plotter.show_bounds(grid='back', location='outer', color='#000000', bounds=self.bounds,
                                 xtitle='x [m]', ytitle='y [m]', ztitle='z [m]')
        self.plotter.view_isometric()

        self.step = step
        self.ts = ts
        self.thread = Thread(target=self.simulation)
        self.thread.start()

    def simulation(self):
        for k in range(0, len(self.x1), self.step):
            if self.isTrajectory:
                self.plotTrajectory(self.hx[k], self.hy[k], k)
            self.robotUniciclo(self.x1[k], self.y1[k], self.phi[k], k)
            time.sleep(self.ts)

    def robotUniciclo(self, x1, y1, phi, k):
        # Rotation matrix for robot orientation (around the Z-axis)
        Rz = np.array([
            [np.cos(phi), -np.sin(phi), 0],
            [np.sin(phi), np.cos(phi), 0],
            [0, 0, 1]
        ])

        # Apply rotation and translation to each robot part
        for i in range(len(self.robotCopy)):
            # Apply rotation to the robot's points
            self.robotCopy[i].points = (Rz @ self.robot[i].points.T).T

            # Apply translation in-place
            self.robotCopy[i].translate([x1, y1, 0], inplace=True)

    def plotTrajectory(self, hx, hy, k):
        self.spline.points[k:self.sizeh, 0] = hx
        self.spline.points[k:self.sizeh, 1] = hy
