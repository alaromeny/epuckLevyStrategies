# Supervisor Object
This script is responsible for the following:

- records the positions of each robot every second, in order to track coverage.
- records a hitting times matrix (time for each cell to first be covered) for the entire area
- draws a trace of each robots movement on the floor to visualise coverage
- resets the experiment after each run


#### Important parameters
This is the number of simulation minutes you would like each run to last
```SIMULATION_RUN_TIME```

You need to specify the number of robots in the simulation. This has to match the number of robotics in the world file you have chosen.
Your options are 5,10,15 or 20
```NUMBER_OF_ROBOTS```

All this does is set the starting number for the output files.
This is useful if webots crashes midway through a batch, you can restart at the last number to make batch use of the data easier.
```SIMULATION_RUN_COUNTER```
