import numpy as np

from read_and_separate import separate_data

# Error parameters
Q = np.array([[0.025, 0, 0],
              [0, 0.0025, 0],
              [0, 0, 0.0025]])

alpha1 = alpha2 = alpha3 = alpha4 = (0.000000001)

def prediction_step(mu, sigma, dt, v, omega, alpha1, alpha2, alpha3, alpha4):
    # Motion model
    theta = mu[2]
    A = np.array([[1, 0, -v/omega * np.sin(theta) + v/omega * np.sin(theta + omega*dt)],
                  [0, 1, v/omega * np.cos(theta) - v/omega * np.cos(theta + omega*dt)],
                  [0, 0, 1]])
    B = np.array([[(-np.sin(theta) + np.sin(theta + omega*dt))/omega, v * (np.sin(theta) - np.sin(theta + omega*dt))/(omega**2) + v * np.cos(theta + omega*dt) * dt],
                  [(np.cos(theta) - np.cos(theta + omega*dt))/omega, -v * (np.cos(theta) - np.cos(theta + omega*dt))/(omega**2) + v * np.sin(theta + omega*dt) * dt],
                  [0, dt]])
    mu_bar = np.dot(A, mu) + np.dot(B, np.array([v, omega]))

    # Calculate R (noise covariance matrix)
    v_hat = v + np.random.normal(0, np.sqrt(alpha1*v**2 + alpha2*omega**2))
    omega_hat = omega + np.random.normal(0, np.sqrt(alpha3*v**2 + alpha4*omega**2))

    G = np.array([[1, 0, -v_hat/omega_hat * np.cos(theta) + v_hat/omega_hat * np.cos(theta + omega_hat*dt)],
                  [0, 1, -v_hat/omega_hat * np.sin(theta) + v_hat/omega_hat * np.sin(theta + omega_hat*dt)],
                  [0, 0, 1]])
    
        # Apply process noise to the covariance matrix
    R = np.array([[alpha1*v**2 + alpha2*omega**2, 0, 0],
                  [0, alpha3*v**2 + alpha4*omega**2, 0],
                  [0, 0, 0]])  # Process noise covariance matrix
    
    sigma_bar = np.dot(np.dot(G, sigma), G.T) + np.dot(np.dot(A, R), A.T) #change to R for alpha effect, keep Q for det(sigma)
    return mu_bar, sigma_bar


# Correction step
def correction_step(mu, sigma, z):
    # Measurement model
    hx = np.array([mu[0], mu[1], mu[2]])
    H = np.eye(3)  # Measurement Jacobian, representing C as the identity matrix
    K = np.dot(np.dot(sigma, H.T), np.linalg.inv(np.dot(np.dot(H, sigma), H.T) + Q))
    mu_updated = mu + np.dot(K, (z - hx))
    sigma_updated = np.dot((np.eye(3) - np.dot(K, H)), sigma)
    return mu_updated, sigma_updated

# EKF localizer function
def ekf_localizer(sensor_data, command_data, alpha1, alpha2, alpha3, alpha4):
    mu = np.array(sensor_data[0][1:])  # Initialize state estimate using the first sensor reading
    sigma = np.zeros((3, 3))  # Initialize covariance matrix
    dt = 0.01  # Time step
    
    poses = [(mu[0], mu[1], mu[2])]  # List to store pose estimates
    determinant_list = []  # List to store determinants of sigma
    
    sensor_idx = 1
    for t in np.arange(0, sensor_data[-1][0], dt):
        # Get the current command data
        if t <= command_data[-1][0]:
            v, omega = command_data[-1][1:]  # Assuming command_data has velocities and omega at index 1 and 2
        
        # Perform prediction step
        mu, sigma = prediction_step(mu, sigma, dt, v, omega, alpha1, alpha2, alpha3, alpha4)
        determinant_list.append(np.linalg.det(sigma))  # Calculate determinant and store it
        poses.append((mu[0], mu[1], mu[2]))  # Store the predicted pose estimate
        
        # Check if there is new sensor data
        if sensor_idx < len(sensor_data) and t >= sensor_data[sensor_idx][0]:
            z = np.array(sensor_data[sensor_idx][1:])
            mu, sigma = correction_step(mu, sigma, z)
            determinant_list.append(np.linalg.det(sigma))  # Calculate determinant and store it
            poses.append((mu[0], mu[1], mu[2]))  # Store the corrected pose estimate
            sensor_idx += 1
        
    return poses, determinant_list


def save_to_txt(poses, output_file):
    with open(output_file, 'w') as file:
        file.write("Poses:\n")
        for pose in poses:
            formatted_pose = ', '.join(map(str, pose))  # Convert tuple to comma-separated string
            file.write(f"{formatted_pose}\n")

def save_to_txt_det(determinant_list, output_file):
    with open(output_file, 'w') as file:
        file.write("Determinants of Sigma:\n")
        for determinant in determinant_list:
            file.write(f"{str(determinant)}\n")

# Example usage
file_path = 'C:\Work\Python\Assignment 1\Data\LandTamerGravel1_raw.data'
command_data, sensor_data = separate_data(file_path)

poses, determinant_list = ekf_localizer(sensor_data, command_data, alpha1, alpha2, alpha3, alpha4)
print(poses)
#print(determinant_list)

# Save data to txt file
output_file = r'C:\Work\Python\Assignment 1\txt\Gravel1_raw\poses_1e-9.txt'
save_to_txt(poses, output_file)
print(f"Data saved to {output_file}")

# Save data to txt file
#output_file = r'C:\Work\Python\Assignment 1\txt\Gravel1_raw\det.txt'
#save_to_txt_det(determinant_list, output_file)
#print(f"Data saved to {output_file}")

