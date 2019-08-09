#!/usr/bin/env python3
"""supervisor_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import Robot
from controller import Display
from controller import Node
from controller import Supervisor
from controller import Emitter
# from scipy.ndimage import gaussian_filter
import numpy as np
import math
import time



class levySupervisor(Supervisor):

    #This is the number of simulation minutes you would like the run to last
    SIMULATION_RUN_TIME = 1
    #Specify the number of robots in the simulation. This has to match the number of robotics in the sim world.
    NUMBER_OF_ROBOTS = 20
    #Run number, should set to 0, but can be set higher. For example if a batch crashes can restart where it stopped.
    #All this does is number the files so you don't have to worry too much
    SIMULATION_RUN_COUNTER = 0


    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks
    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks
    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks
    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks
    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks
    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks
    ## DO NOT MODIFY ANYTHING BELOW THIS LINE, Thanks


    #These are the colours (hex) for the sim environment
    BLACK      = 0x000000
    LIGHT_GRAY = 0x505050
    RED        = 0xBB2222
    BLUE       = 0x2222BB
    GREEN      = 0x22BB11
    WHITE      = 0xFFFFFF

    EVAPORATION_FACTOR = 0.9
    EVAPORATION_TIMESTEP = 640
    # DIFFUSION_TIMESTEP = 1280
    DIFFUSION_TIMESTEP = 64
    GAUSSIAN_SIGMA = 1
    SIMULATION_RUNTIME_TOTAL = 64 * 60 * 20
    # create the Robot instance.
    TIMESTEP_DURATION = 1/64
    


    #simulation minutes * seconds * timesteps per second
    SIMULATION_RUN_LIMIT = SIMULATION_RUN_TIME * 60 * 64 

    robotNumberIndex = (NUMBER_OF_ROBOTS/5)-1

                      #5 robots
    initPosStore = [[[ 0.00585, 0.00095, -0.09039],
                     [ 0.09384, 0.00096,  0.00193],
                     [-0.09467, 0.00096, -0.02299],
                     [ 0.05010, 0.00096,  0.09329],
                     [-0.05967, 0.00096,  0.08318]],
                      #10 robots
                    [[ 0.02135, 0.00084, -0.14686],
                     [ 0.12653, 0.00102, -0.12148],
                     [-0.09024, 0.00099, -0.12278],
                     [ 0.18913, 0.00093, -0.03754],
                     [ 0.18435, 0.00108,  0.06049],
                     [ 0.13409, 0.00119,  0.14923],
                     [ 0.03541, 0.00117,  0.17968],
                     [-0.07120, 0.00109,  0.15393],
                     [-0.14744, 0.00105,  0.08147],
                     [-0.14999, 0.00103, -0.03000]],
                      #15 robots
                    [[-0.00558, 0.00096, -0.25965],
                     [ 0.08711, 0.00096, -0.24787],
                     [ 0.17057, 0.00096, -0.18984],
                     [ 0.21730, 0.00096, -0.10868],
                     [ 0.22201, 0.00096, -0.01970],
                     [ 0.20081, 0.00096,  0.07357],
                     [ 0.14955, 0.00096,  0.14950],
                     [ 0.06142, 0.00097,  0.18151],
                     [-0.03231, 0.00097,  0.18352],
                     [-0.12767, 0.00096,  0.16503],
                     [-0.19320, 0.00096,  0.09200],
                     [-0.22007, 0.00096,  0.00302],
                     [-0.21827, 0.00096, -0.09283],
                     [-0.18342, 0.00096, -0.18111],
                     [-0.09604, 0.00096, -0.23399]],
                     #20 robots
                    [[ 0.00812, 0.00096, -0.30881],
                     [ 0.09189, 0.00096, -0.28587],
                     [ 0.17007, 0.00096, -0.24133],
                     [ 0.23118, 0.00096, -0.17338],
                     [ 0.28061, 0.00096, -0.09903],
                     [ 0.30810, 0.00096, -0.01768],
                     [ 0.31074, 0.00096,  0.06985],
                     [ 0.25374, 0.00097,  0.14464],
                     [ 0.19959, 0.00097,  0.21270],
                     [ 0.12739, 0.00096,  0.26252],
                     [ 0.02980, 0.00096,  0.27346], 
                     [-0.07096, 0.00096,  0.26140],
                     [-0.17201, 0.00096,  0.20537],
                     [-0.24305, 0.00096,  0.14486],
                     [-0.29499, 0.00096,  0.07303],
                     [-0.29397, 0.00096, -0.01894],
                     [-0.27711, 0.00096, -0.10198],
                     [-0.23214, 0.00096, -0.18261],
                     [-0.16320, 0.00096, -0.25246],
                     [-0.07718, 0.00096, -0.28823]]]
 
    initialPositions = initPosStore[robotNumberIndex]



                     #5 robots
    initRotStore = [[[-0.00005, -1, -0.00011,  2.94096],
                     [ 0.00049, -1,  0.00099,  1.69792],
                     [ 0.00049, -1,  0.00099,  1.69792],
                     [ 0.00038,  1,  0.00046,  3.98702],
                     [ 0.00018, -1, -0.00069, -2.61796]],
                     #10 robots
                    [[-0.01299,  1,  0.00423,  0.06116],
                     [ 0.00076, -1,  0.00118,  0.78160],
                     [ 0.00035,  1, -0.00022,  0.74343],
                     [-0.00072, -1, -0.00132,  1.37988],
                     [-0.00012,  1,  0.00085,  4.58149],
                     [ 0.00026, -1, -0.00094, -3.53429],
                     [ 0.00026, -1, -0.00094,  3.10000],
                     [ 0.00026, -1, -0.00094, -2.35616],
                     [ 0.00026, -1, -0.00094, -1.96346],
                     [ 0.00026, -1, -0.00094, -1.57076]],
                     #15 robots
                    [[ 0.00086,  1,  0.01240, 0.13470],
                     [-0.00008,  1,  0.00025, 5.71761],
                     [-0.00081, -1, -0.00113, 1.37988],
                     [ 0.00037, -1, -0.00142, 1.04236],
                     [ 0.00113, -1, -0.00115, 1.54965],
                     [ 0.00009,  1, -0.00092, 3.97802],
                     [-0.00084,  1,  0.00035, 3.92695],
                     [-0.00107,  1,  0.00087, 3.27242],
                     [-0.00185,  1, -0.00040, 3.14094],
                     [-0.00010,  1, -0.00004, 2.48705],
                     [ 0.00003, -1,  0.00002, 4.31972],
                     [-0.00001,  1, -0.00007, 1.43989],
                     [ 0.00001,  1, -0.00006, 1.57079],
                     [ 0.00008, -1, -0.00123, 4.84332],
                     [ 0.99999,  0, -0.00336, 0.00000]],
                     #20 robots
                    [[ 0.00086,  1,  0.01240,  0.13470],
                     [-0.00008,  1,  0.00025,  5.71761],
                     [-0.00081, -1, -0.00113,  1.37988],
                     [-0,       -1, -0.00103,  1.56595],
                     [ 0.00113, -1, -0.00115,  1.54965],
                     [-0.00059, -1,  0.00108,  1.51975],
                     [-0.00084,  1,  0.00035,  3.92695],
                     [-0.00147,  1,  0.00044,  4.05781],
                     [-0.00174,  1, -0.00089,  3.66454],
                     [-0.00007,  1, -0.00007,  3.27245],
                     [ 0,       -1,  0.00002,  3.01073], # [9.716898943246472e-06, -1, 2.8649111614200917e-05, 3.01073]
                     [ 0,        1, -0,        2.61799],
                     [ 0.00002,  1, -0.00004,  2.35619],
                     [ 0.00040, -1, -0.00081, -2.22525],
                     [ 0,        1,  0,        1.83259],
                     [ 0.00001,  1, -0.00005,  1.70169],
                     [ 0.00003,  1, -0.00011,  1.04719],
                     [ 0.00010,  1, -0.00018,  0.78539],
                     [-0.00014, -1, -0.00002,  5.36688],
                     [-0.00006,  1, -0.00006,  0.26179]]]



    initialRotations = initRotStore[robotNumberIndex]



    #This is the name of the file where the data will be output
    #example Robots_5_Run_100.txt
    fileName = "Robots_" + str(NUMBER_OF_ROBOTS) + "_Run_" + str(SIMULATION_RUN_COUNTER) + ".txt"
    loggingFile = open(fileName,"w")
    debuggingFiles = open("Log.txt","w")

    # get the time step of the current world.


    def transformArenaToImage(self, x_robot, y_robot, x_transform, y_transform):
        newX = (x_transform + (x_robot*100))
        newY = (y_transform + (y_robot*100))
        return int(newX), int(newY) 

    def initialization( self):
        self.timestep = int(self.getBasicTimeStep())
        self.goundDisplay = Display("ground_display")
        self.emitter = self.getEmitter('emitter')
        channel = self.emitter.getChannel()

        self.robotList = []

        for i in range(self.NUMBER_OF_ROBOTS):
            robotDef = "levyRobot" + str(i)
            tmp = self.getFromDef(robotDef)
            self.robotList.append(tmp)
            tmp1 = tmp.getField("translation")
            tmp2 = tmp.getField("rotation")

        self.robotFieldList = []
        self.populateRobotFields()
        self.reloadAllRobotPos()

    def reloadAllRobotPos( self):
        print "[reloadAllRobotPos] Setting all robots to start positions"
        waitTime = self.TIMESTEP_DURATION / (self.NUMBER_OF_ROBOTS)
        for i in range(self.NUMBER_OF_ROBOTS):
            myField = self.robotFieldList[i]
            myPos = self.initialPositions[i]
            myRot = self.initialRotations[i]
            myField[0].setSFVec3f(myPos)
            myField[1].setSFRotation(myRot)
            time.sleep(waitTime) #This is a hack to allow webots time between resetting robots to prevent robots crashing during reset


    def populateRobotFields( self):
        for i in range(self.NUMBER_OF_ROBOTS):
            myBot = self.robotList[i]
            myField = []
            myField.append( myBot.getField("translation"))
            myField.append( myBot.getField("rotation"))
            self.robotFieldList.append(myField)


    def resetField( self):
        print "Run " + str(self.SIMULATION_RUN_COUNTER) + " over, time to reset"
        self.simulationSetMode(self.SIMULATION_MODE_PAUSE)

        coverageFileName = "Robots_" + str(self.NUMBER_OF_ROBOTS) + "_Run_" + str(self.SIMULATION_RUN_COUNTER) + "_coverageMatrix.txt"


        np.savetxt(coverageFileName, self.timingsMap, delimiter=',') 


        self.SIMULATION_RUN_COUNTER = self.SIMULATION_RUN_COUNTER + 1

        self.fileName = "Robots_" + str(self.NUMBER_OF_ROBOTS) + "_Run_" + str(self.SIMULATION_RUN_COUNTER) + ".txt"
        self.loggingFile = open(self.fileName,"w")
        self.stigmergyMap = np.zeros((self.automotaMap_width, self.automotaMap_height), dtype=np.uint8)
        self.timingsMap = np.zeros((self.automotaMap_width, self.automotaMap_height), dtype=np.uint32)
        self.automotaMap = np.zeros((self.automotaMap_width, self.automotaMap_height), dtype=np.uint8)
        self.resetGroundDisplay()
        self.reloadAllRobotPos()
        self.simulationSetMode(self.SIMULATION_MODE_FAST)


    def resetGroundDisplay( self):
        self.automotaMap_x, self.automotaMap_y = np.nonzero(self.automotaMap)
        for i in range(len(self.automotaMap_x)):
            x = self.automotaMap_x[i]
            y = self.automotaMap_y[i]
            pheromoneStrength = self.automotaMap[x,y]
            pheromoneColour = int(round((pheromoneStrength * self.WHITE)/255))
            self.goundDisplay.setColor(pheromoneColour)
            self.goundDisplay.fillRectangle(int(x*10), int(y*10), 10, 10)
            # self.goundDisplay.fillRectangle(90, 80, 10, 10)
            self.automotaMap[x,y] = np.uint8(pheromoneStrength)
            self.stigmergyMap[x,y] = np.uint8(1)

    def updateTrail( self, newPos, count):
        minBoundX = int(math.floor(newPos[0] - 3))
        maxBoundX = int(math.floor(newPos[0] + 4))
        minBoundY = int(math.floor(newPos[1] - 3))
        maxBoundY = int(math.floor(newPos[1] + 4))

        if minBoundX < 1:
            minBoundX = 1
        if minBoundY < 1:
            minBoundY = 1
        if maxBoundX >=self.automotaMap_width:
            maxBoundX = self.automotaMap_width-1
        if maxBoundY > self.automotaMap_height:
            maxBoundy = self.automotaMap_height-1

        self.automotaMap[minBoundX:maxBoundX, minBoundY:maxBoundY] = np.uint8(255)
        timestamp = self.timingsMap[minBoundX:maxBoundX, minBoundY:maxBoundY]
        for i in range(0,maxBoundX-minBoundX):
            for j in range(0,maxBoundY-minBoundY):
                if timestamp[i,j] == 0:
                    self.timingsMap[i,j] = count

    def mainLoop( self):
        width = self.goundDisplay.getWidth()
        height = self.goundDisplay.getHeight()

        self.automotaMap_width  = int(width)
        self.automotaMap_height = int(height)
        automotaTiles = (self.automotaMap_width*self.automotaMap_height)

        self.stigmergyMap = np.zeros((self.automotaMap_width, self.automotaMap_height), dtype=np.uint8)
        self.timingsMap = np.zeros((self.automotaMap_width, self.automotaMap_height), dtype=np.uint32)
        self.automotaMap = np.zeros((self.automotaMap_width, self.automotaMap_height), dtype=np.uint8)

        x_transform = width/2;
        y_transform = height/2;



        ##Set up the display to the arena
        self.goundDisplay.setColor(self.BLACK)
        self.goundDisplay.fillRectangle(0, 0, width, height)
        self.goundDisplay.setColor(self.RED)
        self.goundDisplay.setColor(self.BLUE)
        self.goundDisplay.setOpacity(1)
        self.goundDisplay.setColor(self.WHITE)



        # Main loop:  
        count = 0
        while self.step(self.timestep) != -1:

            for i in range(self.NUMBER_OF_ROBOTS):
                myBot = self.robotList[i]
                pos = myBot.getPosition()
                newPos = self.transformArenaToImage(pos[0], pos[2], x_transform, y_transform)
                self.updateTrail(newPos, count)

            if count == self.SIMULATION_RUN_LIMIT: #== 0 and count > 0:
                self.resetField()
                self.goundDisplay.setColor(self.BLACK)
                self.goundDisplay.fillRectangle(0, 0, width, height)
                self.goundDisplay.setColor(self.RED)
                self.goundDisplay.setColor(self.BLUE)
                self.goundDisplay.setOpacity(1)
                self.goundDisplay.setColor(self.WHITE)
                count = 0


            if count % 64 == 0:
                self.automotaMap_x, self.automotaMap_y = np.nonzero(self.automotaMap) 
                for i in range(len(self.automotaMap_x)):
                    x = self.automotaMap_x[i]
                    y = self.automotaMap_y[i]
                    self.goundDisplay.setColor(self.WHITE)
                    self.goundDisplay.fillRectangle(int(x), int(y), 1, 1)
                    self.goundDisplay.fillRectangle(0, 0, 1, 1)
                    self.stigmergyMap[x,y] = np.uint8(1)

                self.stigmergyMap_x, self.stigmergyMap_y = np.nonzero(self.stigmergyMap)
                coverage = float(len(self.stigmergyMap_x)) / float(automotaTiles)
                printMessage = str(coverage) + ",\n"
                self.loggingFile.write(printMessage) 

            count = count + 1
            pass


controller = levySupervisor()
controller.initialization()
controller.mainLoop()