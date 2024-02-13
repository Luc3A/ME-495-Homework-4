import Buginator
import xml.etree.ElementTree as ET 
import mujoco as mj
import mujoco_viewer
import numpy as np
import random
import math

# Initialize arrays to save parent information
generationalSegments = np.array([])
generationalWiggleTorque = np.array([])
generationalStompTorque = np.array([])
generationalFitness = np.array([])

# Fitness multipliers
weightMultiplier = 0.1
distanceMultiplier = 2

# Create random initial traits
numSegments = random.randint(1,10)
np.append(generationalSegments, numSegments)

wiggleTorque = random.randint(30,70)
np.append(generationalWiggleTorque, wiggleTorque)

stompTorque = random.randint(30,70)
np.append(generationalStompTorque, stompTorque)

# iterate 4 generations
for i in range(3):
    # generate xml file and motor torques
    motorList = Buginator.createXML(numSegments)
    stepList = Buginator.createSteps(motorList, wiggleTorque, stompTorque)

    [pos1, pos2] = Buginator.animate(stepList)
    parentFitness = Buginator.calcFitness(numSegments, weightMultiplier, distanceMultiplier, pos1, pos2)
    np.append(generationalFitness, parentFitness)


    #mutate parent traits
    numSegments, wiggleTorque, stompTorque = Buginator.mutate(numSegments, wiggleTorque, stompTorque )
    
    # record new generations traits 
    np.append(generationalSegments, numSegments)
    np.append(generationalWiggleTorque, wiggleTorque)
    np.append(generationalStompTorque, stompTorque)


