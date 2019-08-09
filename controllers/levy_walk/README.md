# e-puck controller
This file contains the controller for the e-puck robot

It:

- generates a new position to navigate to based on a Levy Distribution in 2D space
- manages the robot's states between rotating, moving forward and avoiding obstacles


#### Important parameters
This changes the Levy parameter mu affecting the distribution of Levy flights vs brownian motion
```double global_mu```

This defines the threashold at which an IR sensor detects an obstacle
```#define THRESHOLD_DIST```

This sets the speed of the robots between 0 and 1.
Set to 0.5 and I advise not changing this especially when running experiments with the real epuck
```#define SPEED_SETTING ```
