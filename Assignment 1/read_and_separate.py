def separate_data(file_path):
    command_data = []
    sensor_data = []

    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split(',')
            
            if data[1] != 'NaN' :
                # Command Data Format: (Timestamp, Speed, Curvature, NaN, NaN, NaN, NaN)
                timestamp, speed, curvature = float(data[0]), float(data[1]), float(data[2])
                command_data.append((timestamp, speed, curvature))
                
            else:
                # Sensor Data Format: (Timestamp, NaN, NaN, x, y, z, Roll, Pitch, Yaw)
                timestamp, x, y, Yaw = map(float, (data[0], data[3], data[4], data[8]))
                sensor_data.append((timestamp, x, y, Yaw))
                
    return command_data, sensor_data

def save_to_txt(command_data, sensor_data, output_file):
    with open(output_file, 'w') as file:
        file.write("Command Data:\n")
        for item in command_data:
            formatted_item = ', '.join(map(str, item))  # Convert tuple to comma-separated string
            file.write(f"{formatted_item}\n")
        file.write("----------------------------------------------------------------------------------------------------\n")
        file.write("Sensor Data:\n")
        for item in sensor_data:
            formatted_item = ', '.join(map(str, item))  # Convert tuple to comma-separated string
            file.write(f"{formatted_item}\n")


# Example usage
file_path = r'C:\Work\Python\Assignment 1\Data\LandTamerGravel1_raw.data'
command_data, sensor_data = separate_data(file_path)

# Print separated data
print("Command Data:")
print(command_data)
print("----------------------------------------------------------------------------------------------------")
print("\nSensor Data:")
print(sensor_data)

# Save data to txt file
output_file = r'C:\Work\Python\Assignment 1\txt\Gravel1_raw\Gravel1_raw_separated.txt'
save_to_txt(command_data, sensor_data, output_file)
print(f"Data saved to {output_file}")