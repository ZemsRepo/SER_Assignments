"""
This example demonstrates many of the 2D plotting capabilities
in pyqtgraph. All of the plots may be panned/scaled by dragging with 
the left/right mouse buttons. Right click on any plot to show a context menu.
"""

from typing import Iterable
import numpy as np
import scipy as sp
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore


class Animation:
    def __init__(self, landmarks: Iterable[float]) -> None:
        # window
        self.win = pg.plot(show=True)
        self.win.resize(800, 600)
        self.win.setWindowTitle("Animation Ex03 - x1 speed")
        self.win.setBackground("w")
        self.win.setAspectLocked(lock=True, ratio=1)
        self.win.getViewBox().wheelEvent = lambda event: None
        self.win.setXRange(-2, 10)
        self.win.setYRange(-3, 4)

        # colors
        color_landmarks = pg.mkColor(10, 10, 10)  # black
        color_landmarks.setAlpha(255)

        color_pos_true = pg.mkColor(0, 114, 189)  # blue
        color_pos_true.setAlpha(220)

        color_pos_est = pg.mkColor(252, 41, 30)  # red
        color_pos_est.setAlpha(220)

        color_ellipse = pg.mkColor(252, 41, 30)  # red
        color_ellipse.setAlpha(60)

        # landmarks
        self.landmarks = pg.ScatterPlotItem()
        self.landmarks.setPen(color_landmarks)
        self.landmarks.setBrush(color_landmarks)
        self.landmarks.setData(x=landmarks[:, 0], y=landmarks[:, 1])

        # true position
        self.pos_true = pg.ScatterPlotItem()
        self.pos_true.setSize(5)
        self.pos_true.setPen(color_pos_true)
        self.pos_true.setBrush(color_pos_true)

        # estimated position
        self.pos_est = pg.ScatterPlotItem()
        self.pos_est.setSize(5)
        self.pos_est.setPen(color_pos_est)
        self.pos_est.setBrush(color_pos_est)

        # uncertainty ellipse
        self.ellipse = pg.PlotCurveItem()
        self.ellipse.setPen(color_ellipse, width=4)
        self.ellipse.setBrush(color_ellipse)
        self.ellipse.setFillLevel(0)
        
        # legend
        self.legend = pg.LegendItem()
        self.legend.setParentItem(self.win.graphicsItem())
        self.legend.setOffset((20, 1))
        self.legend.addItem(self.landmarks, "Landmarks")
        self.legend.addItem(self.pos_true, "True position")
        self.legend.addItem(self.pos_est, "Estimated position")
        self.legend.addItem(self.ellipse, "Uncertainty ellipse")
        
        self.win.addItem(self.landmarks)
        self.win.addItem(self.pos_true)
        self.win.addItem(self.pos_est)
        self.win.addItem(self.ellipse)
        
    def compute_ellipse(self, mean: Iterable[float], covariance: Iterable[float]) -> Iterable[float]:
        eigenvalues, eigenvectors = np.linalg.eig(covariance)
        angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
        width, height = 2 * np.sqrt(eigenvalues)
        t = np.linspace(0, 2 * np.pi, 50)
        ell = np.array([width / 2 * np.cos(t), height / 2 * np.sin(t)])
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        rotated_ell = rot_matrix @ ell
        rotated_ell[0] += mean[0]
        rotated_ell[1] += mean[1]
        return rotated_ell
    
    def update(self, mean: Iterable[float], covariance: Iterable[float], true_position = None) -> None:
        if true_position is not None:
            self.pos_true.setData([true_position[0]], [true_position[1]])
            
        self.pos_est.setData([mean[0]], [mean[1]])
        
        ellipse = self.compute_ellipse(mean, covariance)
        self.ellipse.setData(ellipse[0], ellipse[1])


if __name__ == "__main__":
    
    # test 
    l = sp.io.loadmat("ex03/dataset2.mat")["l"]
    animation = Animation(l)

    mean = [1, 1]
    cov = [[2, 1], [1, 2]]

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: animation.update(mean, cov, true_position=[mean[0] + 1, mean[1] + 1]))
    timer.start(1000)

    pg.exec()

# # create plot
# win = pg.plot(show=True)
# win.resize(800, 600)
# win.setWindowTitle("Animation Ex03 - x1 speed")
# win.setBackground("w")
# win.setAspectLocked(lock=True, ratio=1)
# win.setXRange(-2, 10)
# win.setYRange(-3, 4)
# win.getViewBox().wheelEvent = lambda event: None  # disable zooming

# # colors
# color_ellipse = pg.mkColor(252, 41, 30)
# color_ellipse.setAlpha(60)

# color_pos_est = pg.mkColor(252, 41, 30)
# color_pos_est.setAlpha(220)

# color_pos_true = pg.mkColor(0, 114, 189)
# color_pos_true.setAlpha(220)

# color_landmarks = pg.mkColor(10, 10, 10)
# color_landmarks.setAlpha(255)

# # elements
# landmarks = pg.ScatterPlotItem()
# landmarks.setSize(6)
# landmarks.setPen(color_landmarks)
# landmarks.setBrush(color_landmarks)
# landmarks.setData(x=l[:, 0], y=l[:, 1])

# ellipse = pg.PlotCurveItem()
# ellipse.setPen(color_ellipse, width=4)
# ellipse.setBrush(color_ellipse)
# ellipse.setFillLevel(0)

# pos_true = pg.ScatterPlotItem()
# pos_true.setSize(5)
# pos_true.setPen(color_pos_true)
# pos_true.setBrush(color_pos_true)

# pos_est = pg.ScatterPlotItem()
# pos_est.setSize(5)
# pos_est.setPen(color_pos_est)
# pos_est.setBrush(color_pos_est)

# win.addItem(ellipse)
# win.addItem(landmarks)
# win.addItem(pos_est)
# win.addItem(pos_true)


# # create uncertainty ellipse
# mean = [0, 0]
# covariance = [[2, 1], [1, 2]]  # Example values

# eigenvalues, eigenvectors = np.linalg.eig(covariance)
# angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
# width, height = 2 * np.sqrt(eigenvalues)
# t = np.linspace(0, 2 * np.pi, 50)
# ell = np.array([width / 2 * np.cos(t), height / 2 * np.sin(t)])
# rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
# rotated_ell = rot_matrix @ ell


# def update():
#     global ellipse, rotated_ell, mean, win
#     ellipse.setData(rotated_ell[0] + mean[0], rotated_ell[1] + mean[1])
#     pos_est.setData([mean[0]], [mean[1]])
#     pos_true.setData([mean[0] + 1], [mean[1] + 1])
#     mean[0] += np.random.uniform(-1, 1)
#     mean[1] += np.random.uniform(-1, 1)
