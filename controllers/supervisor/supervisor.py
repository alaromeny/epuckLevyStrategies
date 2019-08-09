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
    SIMULATION_RUN_TIME = 20
    #Specify the number of robots in the simulation. This has to match the number of robotics in the sim world.
    NUMBER_OF_ROBOTS = 5
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


    #Positions for sim reset between runs
    # 5 ROBOTS
    initialPositions = [[ 0.0058521, 0.000950252, -0.0903932],
                        [ 0.0938443, 0.000968044,  0.0019351],
                        [-0.0946773, 0.000968031, -0.0229992],
                        [ 0.0501006, 0.000968025,  0.0932924],
                        [-0.0596779, 0.000968038,  0.0831854]]

    # 10 ROBOTS
    # initialPositions = [[-0.0069870542839162, 0.000956436807326726, 0.00536741338262958],
    #                     [0.09169094685890906, 0.000963636370597808, 0.013000000000000001],
    #                     [-0.09, 0.0009680176748101052, 0.04],
    #                     [0.15000000000000002, 0.00095875890759212, 0.09000000000000001],
    #                     [0.17, 0.00096641897125096, 0.18],
    #                     [0.13, 0.00096641897125096, 0.27],
    #                     [0.03, 0.00096641897125096, 0.29000000000000004],
    #                     [-0.049999999999999996, 0.00096641897125096, 0.26],
    #                     [-0.12, 0.00096641897125096, 0.2],
    #                     [-0.12, 0.00096641897125096, 0.12000000000000001]]
    # 15 ROBOTS
    # initialPositions = [[0.23, 0.0009680435422944767, 0.06],
    #                     [-0.14, 0.0009680161777295051, 0.035],
    #                     [0.21, 0.0009680435364847889, 0.14],
    #                     [0.21, 0.0009809469312221878, 0.23],
    #                     [0.07, 0.000978240923014807, 0.33],
    #                     [0.16, 0.0009887652520445406, 0.31],
    #                     [-0.02, 0.0009680461900634426, 0.29],
    #                     [-0.13, 0.0009730536508617011, 0.18],
    #                     [-0.15, 0.0009777153754241385, 0.11],
    #                     [-0.11, 0.0009680212968639235, -0.04],
    #                     [-0.03, 0.0009680186842439664, -0.10],
    #                     [0.15, 0.0009680184404515894, -0.09],
    #                     [0.20, 0.0009680189391682009, -0.02],
    #                     [-0.09, 0.000968044065407651, 0.25],
    #                     [0.06, 0.0009680184304146772, -0.11]]



    #rotations for sim reset between runs
    # 5 ROBOTS
    initialRotations = [[-5.09497e-5,  -1, -0.00011982,  2.94096],
                        [ 0.000496675, -1,  0.00099746,  1.69792],
                        [ 0.000496675, -1,  0.00099746,  1.69792],
                        [ 0.000380418,  1,  0.00046610,  3.98702],
                        [ 0.000187121, -1, -0.00069827, -2.61796]]

    # 10 ROBOTS
    # initialRotations = [[0.0025554460373243626, -0.9999966217288687, -0.0004756326314712683, -0.3229626686107096],
    #                     [0.0007629999997847145, -0.9999990091856983, 0.0011829871606500746, 0.7816003061004253],
    #                     [0.0003566364797365051, 0.9999999110113856, -0.0002253611375531424, 0.7434303061004254],
    #                     [-0.0007799191461484137, -0.9999991341819257, -0.001059887505581233, 1.6416803061004253],
    #                     [0.00026381797155092235, -0.9999995178471524, -0.000945888756973186, 1.4398996938995747],
    #                     [0.00026381797155092235, -0.9999995178471524, -0.000945888756973186, -3.5342896938995745],
    #                     [0.00026381797155092235, -0.9999995178471524, -0.000945888756973186, 3.1],
    #                     [0.00026381797155092235, -0.9999995178471524, -0.000945888756973186, -2.3561603061004255],
    #                     [0.00026381797155092235, -0.9999995178471524, -0.000945888756973186, -1.9634603061004252],
    #                     [0.00026381797155092235, -0.9999995178471524, -0.000945888756973186, -1.5707603061004252]]

    # 15 ROBOTS
    # initialRotations = [[0.0009486964126594656, -0.9999992092591607, 0.0008255035857103667, 1.4360999491871043],
    #                     [0.0003451028391979193, 0.9999999236652558, -0.00018323076439567582, 1.0052296874640754],
    #                     [-0.0008250220969100609, -0.9999992674604443, -0.0008856732548828755, 1.6416803856673587],
    #                     [-0.0018789624631880195, -0.9999964883537951, -0.0018688981086108856, 2.35033],
    #                     [-0.0004464467272156978, -0.9999968929184112, -0.0024525168385054245, 3.12],
    #                     [0.0004082126198514164, -0.9999980061910726, -0.001954731781210329, 2.82897],
    #                     [0.000347035335886813, -0.9999995888593013, -0.0008377635132016786, -2.356160452773662],
    #                     [0.0002988501633188131, -0.9999985789374328, -0.0016591599365255996, -1.9634469481248313],
    #                     [0.002336192584204756, -0.9999962202602911, -0.0014497135374835044, -1.570752843209619],
    #                     [0.0005460903144469982, -0.9999998146342802, -0.000269289386254142, -0.9162602582148464],
    #                     [0.0004033416915884511, -0.9999999154413883, -8.020409060907208e-05, -0.39265957133596807],
    #                     [-4.7074712384360354e-05, -0.9999999987635659, -1.6026220165057252e-05, 0.6544994121294365],
    #                     [-0.00019657142476917254, -0.9999999742351088, -0.00011353174374636447, 1.0471994149723456],
    #                     [0.0003970385536799388, -0.9999995971112191, -0.0008050700505360964, -2.2252599133288067],
    #                     [0.9995396548286064, -1.7036996400668154e-07, -0.030339387355075725, 4.6920630404011267e-07]]


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