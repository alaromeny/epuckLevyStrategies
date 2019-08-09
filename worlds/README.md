# World Files
There are 4 premade world file with either 5, 10, 15 or 20 robots.
They are setup where the supervisor uses the supervisor controller and the robots use the levy_walk controller.

The robots start in the centre, in a ring formation pointint ourwards.

If you change these potitions + rotations, you will need to update the supervisor controller as this resets the robots at the start of each run, meaning that your new positions will be overwritten.
