
modelPath = '../../models/1/'

weightPath = '../../models/1/'

with open(modelPath + 'Settings.txt', 'r') as f:
    modelFile = f.readline().replace('\n', '')
    weightFile = f.readline().replace('\n', '')
    f.close()
    modelFile = modelFile[modelFile.find('=') + 1:]
    weightFile = weightFile[weightFile.find('=') + 1:]


modelFilename = modelPath + modelFile + '.pep'

weightFileName = weightPath + weightFile + '.txt'
