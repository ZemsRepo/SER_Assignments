import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Given mean and covariance, we need to create an ellipse without using the Ellipse function.

# Mean and covariance for the example
mean = [0, 0]
covariance = [[2, 1], [1, 2]]  # Example values

# Calculate the eigenvalues and eigenvectors of the covariance matrix
eigenvalues, eigenvectors = np.linalg.eig(covariance)

# Calculate the angle for the ellipse from the eigenvectors
# angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
print(angle)

# Calculate width and height of the ellipse using eigenvalues (the lengths of the axes are related to the eigenvalues)
width, height = 2 * np.sqrt(eigenvalues)

# Parametric equation of ellipse
t = np.linspace(0, 2 * np.pi, 100)
ell = np.array([width / 2 * np.cos(t), height / 2 * np.sin(t)])
rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], 
                       [np.sin(angle), np.cos(angle)]])
rotated_ell = np.dot(rot_matrix, ell)

# Plotting the ellipse
plt.plot(rotated_ell[0] + mean[0], rotated_ell[1] + mean[1], color='red')
plt.scatter(mean[0], mean[1], color='blue') # center of ellipse
plt.title("Ellipse from Mean and Covariance")
plt.grid(color='lightgray', linestyle='--')
plt.axis('equal')
plt.show()