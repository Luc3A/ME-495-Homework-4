import xml.etree.ElementTree as ET 
import mujoco as mj
import mujoco_viewer
import numpy as np
import random
import math

def createXML(numSegments):
    # create elements/subelements
    mujoco = ET.Element('mujoco')

    option = ET.SubElement(mujoco, 'option')
    option.text = 'timestep="0.1"'

    worldbody = ET.SubElement(mujoco, 'worldbody')
    actuator = ET.SubElement(mujoco, 'actuator')


    # create floor
    floorSize = 100
    light = ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 1")
    floor = ET.SubElement(worldbody, 'geom', name="floor", type="plane", size=f"{floorSize} {floorSize} 0.1", rgba="0 .39 0 1")


    # make the head
    bodyFrame = ET.SubElement(worldbody, 'body', name="head")
    freeJoint = ET.SubElement(bodyFrame, 'freejoint', name="root")
    head = ET.SubElement(bodyFrame, 'geom', name="head", pos="0 0 0.4", type="box", size="0.3 0.2 0.2", rgba="1 .75 0 1")


    # create an empty array for all motor types (legs or body to actuate later) 
    motorList = np.array([])

    # for each body segments, create a body and attach legs
    for i in range(numSegments):  
        MotorGear = f"100"
        bodyName = f'body{i}Body'
        jointName = f"body{i}Joint"
        hingeName = f"body{i}Hinge"
        bodyBody = ET.SubElement(bodyFrame, 'body', name = bodyName, pos = f'{(i+1)*0.65} 0 0.4')
        bodyGeom = ET.SubElement(bodyBody, 'geom', type = "box", size = "0.3 0.2 0.2", rgba = "0.75 1 0 1")
        bodyHinge = ET.SubElement(bodyBody, 'joint', name=jointName, pos = "-0.15 0 0", axis = "0 0 1", range = "-20 20", limited ="true")
        bodyMotor = ET.SubElement(actuator, 'motor', name=hingeName, gear="10", joint=jointName)

        motorList = np.append(motorList, 1)

        #add legs to the body
        numLegs = 4
        motorList = np.append(motorList, numLegs)

        leg1Name = f"body{i}leg1"
        leg1Hinge = f"body{i}leg1Hinge"
        leg1Body = ET.SubElement(bodyBody, 'body', name= leg1Name)
        leg1Geom = ET.SubElement(leg1Body, 'geom', pos= f'0.2 0.35 0.05', type = 'box', size = '0.05 0.15 0.05')
        leg1Joint = ET.SubElement(leg1Body, 'joint', name = leg1Hinge, pos="0 0 0.5", axis="1 0 0", range="-10 10", limited="true")
        leg1Motor = ET.SubElement(actuator, 'motor', name=leg1Hinge, gear=MotorGear, joint=leg1Hinge)

        leg11Name = f"body{i}leg11"
        leg11Hinge = f"body{i}leg11Hinge"
        leg11Body = ET.SubElement(bodyBody, 'body', name= leg11Name)
        leg11Geom = ET.SubElement(leg1Body, 'geom', pos= f'0.2 0.45 -.1', type = 'box', size = '0.05 0.05 0.2')

        leg2Name = f"body{i}leg2"
        leg2Hinge = f"body{i}leg2Hinge"
        leg2Body = ET.SubElement(bodyBody, 'body', name= leg2Name)
        leg2Geom = ET.SubElement(leg2Body, 'geom', pos= f'-0.2 0.35 0.05', type = 'box', size = '0.05 0.15 0.05')
        leg2Joint = ET.SubElement(leg2Body, 'joint', name = leg2Hinge, pos="0 0 0.5", axis="1 0 0", range="-10 10", limited="true") 
        leg2Motor = ET.SubElement(actuator, 'motor', name=leg2Hinge, gear=MotorGear, joint=leg2Hinge)

        leg21Name = f"body{i}leg21"
        leg21Hinge = f"body{i}leg21Hinge"
        leg21Body = ET.SubElement(bodyBody, 'body', name= leg21Name)
        leg21Geom = ET.SubElement(leg2Body, 'geom', pos= f'-0.2 0.45 -.1', type = 'box', size = '0.05 0.05 0.2')

        leg3Name = f"body{i}leg3"
        leg3Hinge = f"body{i}leg3Hinge"
        leg3Body = ET.SubElement(bodyBody, 'body', name= leg3Name)
        leg3Geom = ET.SubElement(leg3Body, 'geom', pos= f'0.2 -0.35 0.05', type = 'box', size = '0.05 0.15 0.05')
        leg3Joint = ET.SubElement(leg3Body, 'joint', name = leg3Hinge, pos="0 0 0.5", axis="1 0 0", range="-10 10", limited="true")
        leg3Motor = ET.SubElement(actuator, 'motor', name=leg3Hinge, gear=MotorGear, joint=leg3Hinge)
        
        leg31Name = f"body{i}leg31"
        leg31Hinge = f"body{i}leg31Hinge"
        leg31Body = ET.SubElement(bodyBody, 'body', name= leg31Name)
        leg31Geom = ET.SubElement(leg3Body, 'geom', pos= f'0.2 -0.45 -.1', type = 'box', size = '0.05 0.05 0.2')

        leg4Name = f"body{i}leg4"
        leg4Hinge = f"body{i}leg4Hinge"
        leg4Body = ET.SubElement(bodyBody, 'body', name= leg4Name)
        leg4Geom = ET.SubElement(leg4Body, 'geom', pos= f'-0.2 -0.35 0.05', type = 'box', size = '0.05 0.15 0.05')
        leg4Joint = ET.SubElement(leg4Body, 'joint', name = leg4Hinge, pos="0 0 0.5", axis="1 0 0", range="-10 10", limited="true")
        leg4Motor = ET.SubElement(actuator, 'motor', name=leg4Hinge, gear=MotorGear, joint=leg4Hinge)

        leg41Name = f"body{i}leg41"
        leg41Hinge = f"body{i}leg41Hinge"
        leg41Body = ET.SubElement(bodyBody, 'body', name= leg41Name)
        leg41Geom = ET.SubElement(leg4Body, 'geom', pos= f'-0.2 -0.45 -.1', type = 'box', size = '0.05 0.05 0.2')


    # write to XML file 
    tree = ET.ElementTree(mujoco)
    tree.write('bug.xml')
    return motorList


def createSteps(motorList, wiggleTorque, stompTorque):
    step1 = np.array([])
    step2 = np.array([])

    count = 0
    counter = 1

    # create 2 arrays for each step taken by the bug
    for i in motorList:
        if i == 1:
            if counter == 1:
                step1 = np.append(step1, wiggleTorque)
                step2 = np.append(step2, wiggleTorque * -1)

                counter = 2

            if counter == 0:
                step1 = np.append(step1, wiggleTorque * -1)
                step2 = np.append(step2, wiggleTorque)
                counter = 1

            if counter == 2:
                counter = 0


        if i == 4:
            step1 = np.append(step1, [stompTorque/2, stompTorque * -1, stompTorque, stompTorque/2 * -1])
            step2 = np.append(step2, [stompTorque * -1, stompTorque/2, stompTorque/2 * -1, stompTorque])

    stepList = np.array([step1, step2])
    return stepList

def animate(stepList):
    step1 = stepList[0]
    step2 = stepList[1]
    model = mj.MjModel.from_xml_path('bug.xml')
    data = mj.MjData(model)

    viewer = mujoco_viewer.MujocoViewer(model, data)

    motors = model.nu

    data.ctrl[:motors] = step2
    count = 0

    # create empty arrays to record each position
    pos1 = np.array([])
    positions = np.array([])

    # simulate the bugs 
    for i in range(5000):
        if i == 1:
            pos1 = data.body('body0Body').xpos
        if viewer.is_alive:

            if i%50 == 0:
                step = stepList[count]
            
                count = count + 1
                if count == 2:
                    count = 0
            data.ctrl[:motors] = step
            mj.mj_step(model, data)
            viewer.render()
        
        else:
            break
    pos2 = data.body('head').xpos
    viewer.close
    return pos1, pos2
    

# the fitness is determined by both the weight of the bug and the distance it is able to travel
# more weight = needs more sustenance = in a competitive environment it might not be able to eat enough to survive 
# more distance = faster = better at evading theoretical predators = survival! :) 

def calcFitness(numSegments, weightMultiplier, distanceMultiplier, pos1, pos2):
    weight = (numSegments + 1) * weightMultiplier
    distance = abs(distanceMultiplier * (math.sqrt(pos1[0]**2 + pos1[1]**2) - math.sqrt(pos2[0]**2 + pos2[1]**2)))

    # display bug stats 
    print("Weight: ", weight)
    print("Distance: ", math.sqrt(pos1[0]**2 + pos1[1]**2) - math.sqrt(pos2[0]**2 + pos2[1]**2))
    fitness = distance/weight
    print("Fitness: ", fitness)
    return weight, distance, fitness

def mutate(numSegments, wiggleTorque, stompTorque):
    numSegments = numSegments + random.randint(-2,2)
    wiggleTorque = wiggleTorque + random.randint(-10,10)
    stompTorque = stompTorque +  random.randint(-10,10)

    if numSegments < 0:
        numSegments = 0

    return numSegments, wiggleTorque, stompTorque

    

