
import sys
# sys.path.insert(0, 'C:/P Systems/SNPS/pep/')
sys.path.insert(0, '../../utils/pep')
sys.path.insert(0, '../../utils/1')

from pep import readInputFile

from UtilWebots import UtilWebots

from controller import Robot, Motor, PositionSensor
import time
import psutil

from Init import *

# Setup GPS for position tracking
gps = robot.getDevice("gps")
gps.enable(timestep)

# Open file for path coordinates (two columns: x, y)
path_file = open("robot_path_%s.txt" % time.strftime("%d-%m-%Y_%H-%M-%S"), mode="w")


process = psutil.Process()

# Start measuring time
start_time = time.time()
print(f"Start time: {start_time} seconds")
# Get initial memory usage
start_memory = process.memory_info().rss
print(f"Start memory: {start_memory} bytes")
WHEEL_RADIUS = 0.0205  # E-puck wheel radius in meters

# Set motor positions to infinity (velocity control)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Get encoder devices
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

# Enable encoders
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# Initialize variables for distance tracking
total_distance = 0.0
previous_left_distance = 0.0
previous_right_distance = 0.0

# Initialize variables for speed variation (delta_v) tracking
previous_left_speed = None
previous_right_speed = None
speed_variation_sum = 0.0
timestep_count = 0

# Initialize variables for average clearance tracking
clearance_sum = 0.0
clearance_timesteps = 0

# Initialize variables for CPU time per step tracking
cpu_time_sum = 0.0
cpu_step_count = 0

while robot.step(timestep) != -1:
    step_cpu_start = time.process_time()
    # Get GPS position and write to file
    gps_values = gps.getValues()
    path_file.write("%.6f %.6f\n" % (gps_values[0], gps_values[1]))
    path_file.flush()

    # Read encoder values (in radians)
    left_distance = left_encoder.getValue()
    right_distance = right_encoder.getValue()

    # Calculate the distance traveled by each wheel since the last step
    left_wheel_distance = (left_distance - previous_left_distance) * WHEEL_RADIUS
    right_wheel_distance = (right_distance - previous_right_distance) * WHEEL_RADIUS

    # Calculate the average distance traveled by the robot
    average_distance = (left_wheel_distance + right_wheel_distance) / 2.0

    # Update the total distance
    total_distance += average_distance

    # Update previous encoder values
    previous_left_distance = left_distance
    previous_right_distance = right_distance

    # Print the total distance traveled
    #print(f"Total distance traveled: {total_distance:.3f} meters")
    
    
    for sensor in sensors:
        value = utilWebots.get(sensor[1].getValue())
        sensor[0].value = value
    csvFile.write("%d, %s, ,%s\n" % (
        timestep,
        ", ".join([str(var.value) for var in system.variables]),
        ", ".join([str(enz.value) for enz in system.enzymes])))
    system.runSimulationStep()
    

    lw = variables['lw'].value
    rw = variables['rw'].value
    print('lw = {}, rw = {}'.format(lw, rw))
    
    leftMotor.setVelocity(lw)
    rightMotor.setVelocity(rw)

    # Calculate speed variation (delta_v)
    if previous_left_speed is not None and previous_right_speed is not None:
        speed_variation_sum += abs(lw - previous_left_speed) + abs(rw - previous_right_speed)
        timestep_count += 1
    previous_left_speed = lw
    previous_right_speed = rw

    # Calculate and print average speed variation
    if timestep_count > 0:
        delta_v = speed_variation_sum / timestep_count
        print(f"Average speed variation (delta_v): {delta_v:.4f}")

#call GA 
#setarea weighturilor in P sistem 
    # printEnzyme(['eds0', 'eds1', 'eds2', 'eds3'])
    # printEnzyme(['ed', 'eds', 'edw', 'edd'])
    # printVariables(['state', 'angle', 'directionLeft', 'directionRight', 'distance', 'angleStep', 'distanceStep'])
    #printVariables(['lw', 'rw'])

    value = dict(map(lambda o: (o, utilWebots.get(robotSensor[o].getValue())), ['ps5', 'ps6', 'ps7', 'ps0', 'ps1', 'ps2']))
    print(value)

    # Calculate average clearance (only non-zero sensor values, since 0 means no obstacle)
    non_zero_sensors = [v for v in value.values() if v != 0]
    if non_zero_sensors:
        avg_sensor_value = sum(non_zero_sensors) / len(non_zero_sensors)
        clearance_sum += avg_sensor_value
        clearance_timesteps += 1
    if clearance_timesteps > 0:
        average_clearance = clearance_sum / clearance_timesteps
        #print(f"Average clearance (C): {average_clearance:.4f}")

    #printVariables(['weightLeft', 'weightRight'])

    # Calculate CPU time for this step
    step_cpu_time = time.process_time() - step_cpu_start
    cpu_time_sum += step_cpu_time
    cpu_step_count += 1
    avg_cpu_time_per_step = cpu_time_sum / cpu_step_count
    #print(f"CPU time this step: {step_cpu_time * 1000:.4f} ms, Avg CPU time/step: {avg_cpu_time_per_step * 1000:.4f} ms")

    #end_time = time.time()
    #print(f"End time: {end_time-start_time} seconds")

    #end_memory = process.memory_info().rss
    #print(f"End memory: {end_memory-start_memory} bytes")

# Close path file when simulation ends
path_file.close()
