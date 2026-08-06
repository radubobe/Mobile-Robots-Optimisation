import sys
sys.path.insert(0, '../../utils/nsga')

from nsga2.utils import NSGA2Utils
from nsga2.problem_definition import WeightOptimizationProblem
from nsga2.population import Population
from controller import Robot, Motor, PositionSensor
from UtilWebots import UtilWebots
from Util import readData
from LoadModel import modelFilename,weightFileName
from Init_nsga import *
sys.path.insert(0, '../../utils/pep')

from pep import readInputFile
import time
import psutil

process = psutil.Process()

# Start measuring time
start_time = time.time()
#print(f"Start time: {start_time} seconds")
# Get initial memory usage
start_memory = process.memory_info().rss
#print(f"Start memory: {start_memory} bytes")

# Initialize robot and timestep

timestep = int(robot.getBasicTimeStep())

# Setup GPS for position tracking
gps = robot.getDevice("gps")
gps.enable(timestep)

# Open file for path coordinates (two columns: x, y)
path_file = open("robot_path_%s.txt" % time.strftime("%d-%m-%Y_%H-%M-%S"), mode="w")


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

# Initialize GA call counter
ga_call_count = 0

# Initialize problem and GA details
num_of_individuals=50
problem = WeightOptimizationProblem()
utils = NSGA2Utils(
    problem=problem,
    num_of_individuals=50,  # Population size
    mutation_strength=0.5,
    num_of_genes_to_mutate=2,
    num_of_tour_particips=3
)
population = utils.create_initial_population()

#print(population)
# Track previous weights
prev_weights = [0.0] * problem.nr_weights  
prev_sensors_values = [0.0]*(int(problem.nr_weights/2))
prev_lw=0
prev_rw=0
first_step=True
# Main control loop
while robot.step(timestep) != -1:
    #step_cpu_start = time.process_time()
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
    print(f"Total distance traveled: {total_distance:.3f} meters")
    for sensor in sensors:
        value = utilWebots.get(sensor[1].getValue())
        sensor[0].value = value
    csvFile.write("%d, %s, ,%s\n" % (
        timestep,
        ", ".join([str(var.value) for var in system.variables]),
        ", ".join([str(enz.value) for enz in system.enzymes])))
    #system.runSimulationStep()
    
    sensors_values = [utilWebots.get(sensor[1].getValue()) for sensor in sensors]
    #print(sensors_values)
    if first_step==True or any(value != 0 for value in sensors_values):
    #(sensors_values != prev_sensors_values):
        # Increment GA call counter
        ga_call_count += 1
        # Step 1: Run GA for one generation
        utils.fast_nondominated_sort(population)
        for front in population.fronts:
            utils.calculate_crowding_distance(front)

        # Step 2: Get the best weights and update the system
        best_individual = population.fronts[0][0]
        new_weights = best_individual.features
        problem.update_weights(new_weights,variables)
        
        
        #print("After P system step:")
        #print(f"weightLeft: {variables['weightLeft'].value}, weightRight: {variables['weightRight'].value}")
        #print(f"lw: {variables['lw'].value}, rw: {variables['rw'].value}")
        
        # Step 5: Update previous weights
        prev_weights = new_weights
        
        # Step 6: Generate the next generation
        children = utils.create_children(population)
        population.extend(children)
        utils.fast_nondominated_sort(population)
        new_population = Population()
        for front in population.fronts:
            if len(new_population) + len(front) > num_of_individuals:
                front.sort(key=lambda ind: ind.crowding_distance if ind.crowding_distance is not None else -float('inf'), reverse=True)
                new_population.population.extend(front[:num_of_individuals - len(new_population)])
                break
            new_population.population.extend(front)
        population = new_population
        first_step=False
    # Step 3: Run simulation step with updated weights
    system.runSimulationStep()
    # Step 4: Set motor velocities
    lw = variables['lw'].value
    rw = variables['rw'].value
    prev_sensors_values=sensors_values
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

    # Calculate average clearance (only non-zero sensor values, since 0 means no obstacle)
    non_zero_sensors = [v for v in sensors_values if v != 0]
    if non_zero_sensors:
        avg_sensor_value = sum(non_zero_sensors) / len(non_zero_sensors)
        clearance_sum += avg_sensor_value
        clearance_timesteps += 1
    if clearance_timesteps > 0:
        average_clearance = clearance_sum / clearance_timesteps
        print(f"Average clearance (C): {average_clearance:.4f}")

    end_time = time.time()
    print(f"Time: {end_time-start_time} seconds")

    end_memory = process.memory_info().rss
    print(f"End memory: {end_memory-start_memory} bytes")

    # Calculate CPU time for this step
    #step_cpu_time = time.process_time() - step_cpu_start
    #cpu_time_sum += step_cpu_time
    #cpu_step_count += 1
    #avg_cpu_time_per_step = cpu_time_sum / cpu_step_count
    #print(f"CPU time this step: {step_cpu_time * 1000:.4f} ms, Avg CPU time/step: {avg_cpu_time_per_step * 1000:.4f} #ms")
    print(f"GA calls: {ga_call_count}")

# Close path file when simulation ends
path_file.close()
  