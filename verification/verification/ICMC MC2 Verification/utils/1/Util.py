import re


def findVariable(name, system):
    return list(filter(lambda o:o.name == name, system.variables))


def findAllVariable(names, system):
    return list(filter(lambda o:o.name in names, system.variables))


def readData(filename):
    with open(filename, 'r', encoding='UTF-8') as f:
        line = f.readline().replace('\n', '')
        sensorNames = re.split(',\s*', line)
        line = f.readline().replace('\n', '')
        leftWeightNames = re.split(',\s*', line)
        line = f.readline().replace('\n', '')
        rightWeightNames = re.split(',\s*', line)
        weights = {}
        line = f.readline().replace('\n', '')
        while len(line) > 0:
            n = re.split('\s*=\s*', line)
            weights[n[0]] = float(n[1])
            line = f.readline().replace('\n', '')
    return sensorNames, leftWeightNames, rightWeightNames, weights
