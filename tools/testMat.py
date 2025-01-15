import numpy as np

# Define matrices A and B
A = np.array([[0.99818125, 0.00621723, -0.05996276, 0.26695413],
              [0.05996458, -0.00010771, 0.99820050, 0.37697590],
              [0.00619958, -0.99998067, -0.00048033, 0.15036302],
              [0.00000000, 0.00000000, 0.00000000, 1.00000000]])

B = np.array([[0.999123, 0.0418755, -5.483e-5, 0.285651],
              [0.000164466, -0.00523338, -0.999986, -0.662855],
              [-0.0418752, 0.999109, -0.00523568, 0.786325],
              [0, 0, 0, 1]])

# Calculate the product of matrices A and B
C = np.dot(A, B)

# Calculate the inverse of matrix C
C_inv = np.linalg.inv(C)

# Print the result
# Lidar frame to imu frame
print("Lidar frame to imu frame Matrix A * Matrix B is:")
for row in C:
    print(",".join(map(str, row)))
# imu frame to lidar frame
print("\nimu frame to lidar frame Inverse of Matrix C is:")
for row in C_inv:
    print(",".join(map(str, row)))