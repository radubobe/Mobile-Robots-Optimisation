from pep import readInputFile
from UtilWebots import UtilWebots
from Util import readData, findAllVariable
from LoadModel import modelFilename, weightFileName
from controller import Robot
import time
system = readInputFile(modelFilename)

sensorNames, leftWeightNames, rightWeightNames, weights = readData(weightFileName)

modelSensors = dict(map(lambda v:(v.name, v), findAllVariable(sensorNames, system)))
leftWeights = dict(map(lambda v:(v.name, v), findAllVariable(leftWeightNames, system)))
rightWeights = dict(map(lambda v:(v.name, v), findAllVariable(rightWeightNames, system)))

variables = dict(map(lambda o:(o.name, o), system.variables))

enzymes = dict(map(lambda o:(o.name, o), system.enzymes))
csvFile = open("pep_%s.csv" % time.strftime("%d-%m-%Y_%H-%M-%S"), mode="w")

csvFile.write("PeP csv output. Format = STEP_NR VARIABLE_COLUMNS EMPTY_COLUMN ENZYME_COLUMNS\n")
csvFile.write("step, %s, ,%s\n" % (", ".join([var.name for var in system.variables]),", ".join([enz.name for enz in system.enzymes])))
for e in system.enzymes:
    if e.name not in ['eds', 'edw', 'ed', 'edd', 'eds0', 'eds1', 'eds2', 'eds3']:
        e.value = 1000
#de mutat in controller
for weight in weights.keys():
    variables[weight].value = weights[weight]
weight_keys = list(weights.keys())
variables[weight_keys[-1]].value = 1
robot = Robot()

timestep = int(robot.getBasicTimeStep())


leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

camera = robot.getDevice('camera')
camera.enable(timestep)


robotSensor = {}

psNames = list(map(lambda s: 'ps{}'.format(s), range(8)))

for sn in psNames:
    robotSensor[sn] = robot.getDevice(sn)
    robotSensor[sn].enable(timestep)

numbers = robotSensor['ps0'].getLookupTable()

sensors = [(modelSensors['s1'], robotSensor['ps7']),
          (modelSensors['s2'], robotSensor['ps6']),
          (modelSensors['s3'], robotSensor['ps5']),
          (modelSensors['s4'], robotSensor['ps0']),
          (modelSensors['s5'], robotSensor['ps1']),
          (modelSensors['s6'], robotSensor['ps2'])]

utilWebots = UtilWebots(numbers)

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# leftMotor.setVelocity(1)
# rightMotor.setVelocity(0)

# print(ps['ps0'].getLookupTable())

lw, rw = 0, 0


def printVariables(names):
    if isinstance(names, str):
        if names in variables:
            print('{}: {}'.format(names, variables[names].value))
    else:
        n = list(map(lambda o: '{}: {}'.format(o, variables[o].value), names))
        print(', '.join(n))



def printEnzyme(names):
    if isinstance(names, str):
        if names in enzymes:
            print('{}: {}'.format(names, enzymes[names].value))
    else:
        n = list(map(lambda o: '{}: {}'.format(o, enzymes[o].value), names))
        print(', '.join(n))
