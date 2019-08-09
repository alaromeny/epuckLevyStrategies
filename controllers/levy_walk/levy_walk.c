/*
 * File:          brownian_motion.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/led.h>
#include <webots/position_sensor.h> 

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP                 64
#define SPEED_SETTING             0.5
#define NB_LEDS                   8
#define ON                        1
#define OFF                       0
#define DIST_SENS_PAR             70.0
#define THRESHOLD_DIST            100
#define ENCODER_UNIT              159.23
#define AXLE_LENGTH               5.3
#define WHEEL_DIAMETER            4.1
// #define M_PI                      3.14159265358979323846
#define DIST_SENSOR_CALIB_STEPS   50

#define SIMULATION 0       // for robot_get_mode() function
#define CROSSCOMPILATION 1 // for robot_get_mode() function
#define REALITY 2          // for robot_get_mode() function


#define STATE_CALCULATENEXTMOVE 0
#define STATE_ROTATECONTROL 1
#define STATE_FORWARDCONTROL 2
#define STATE_AVOIDOBSTACLE -1

#define NB_DIST_SENS 8
#define PS_RIGHT_10 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_10 7

typedef struct _Vector {
    double u;
    double v;
} Vector;

//This setting allows you to change the Levy parameter mu
double global_mu = 2.1;


double angle_theta, move_parameter_x, move_parameter_y;
double current_maneuver, total_units_rotation, total_units_distance;
double goal_units_rotation, goal_units_distance, distance_d;


double speed_unit = 0.01256 * SPEED_SETTING;

int ps_values[NB_DIST_SENS]={0,0,0,0,0,0,0,0};
int current_state = 0;
int last_state = 0;
int left_motor_speed = 500;
int right_motor_speed = 500;


int ps_offset[NB_DIST_SENS] = {35,35,35,35,35,35,35,35};
int ps_offset_real[NB_DIST_SENS] = {375,158,423,682,447,594,142,360}; // to be modified according to your robot
char * ps_text[]={"one","two","three","five","seven","nine","ten","eleven"};

double left_encoder_offset = 28388;
double sigma = 10.0; 

WbDeviceTag left_motor;
WbDeviceTag right_motor;
WbDeviceTag left_position_sensor;
WbDeviceTag ps[NB_DIST_SENS];
WbDeviceTag led[NB_LEDS];




/************** Obsatcle Detection functions **************
 * These Functions help detect obstacles:
 * The first gathers readings from all sensors
 * Then two o
 **********************************************************/

void readProximitySensors(int ps_values[8], WbDeviceTag ps[8]){
  int i = 0;
  for (i = 0; i < 8 ; i++){
    ps_values[i] = (int)wb_distance_sensor_get_value(ps[i]);
  }
}

bool rightObstacle(){
  bool obstacle =
    (ps_values[0] - ps_offset[0]) > THRESHOLD_DIST ||
    (ps_values[1] - ps_offset[1]) > THRESHOLD_DIST ||
    (ps_values[2] - ps_offset[2]) > THRESHOLD_DIST;

  return obstacle;
}

bool leftObstacle(){
  bool obstacle =
    (ps_values[5] - ps_offset[5]) > THRESHOLD_DIST ||
    (ps_values[6] - ps_offset[6]) > THRESHOLD_DIST ||
    (ps_values[7] - ps_offset[7]) > THRESHOLD_DIST;
  return obstacle;
}


/************** Calibrate function **************
* This function calibrates IR sensors
* This calibration is done over n simulation steps.
* The results are stored in the corresponded offset.
*************************************************/

void calibrateIRSensors(int n){

  int i,it;

  // make sure the sensors are enabled
  wb_robot_step(TIME_STEP);

  // during the following n-1 simulation steps, increment the arrays
  for (it=0; it<n; it++){
      for(i=0;i<NB_DIST_SENS;i++){
        ps_offset[i] += (int) wb_distance_sensor_get_value(ps[i]);
      }
      wb_robot_step(TIME_STEP);
  }

  // devide each element of the array by n-1 in order to have the mean
  for(i=0;i<NB_DIST_SENS;i++){
    ps_offset[i] /= n-1;
  }

}

/**********************************************************
 *
 * These Functions help calculate the motion of the robot
 *
 **********************************************************/

//This calculates the norm between the robot and a new point x,y
static double norm(const Vector *v) {
  return sqrt(v->u*v->u + v->v*v->v);
}

//This calculates the rotation needed to move to a new point x,y from the current frame of reference
static double rotation(const Vector *v) {
  return atan2(v->u,v->v);
}

//This function calculates a random number along a gaussian distribution
double randn (double mu, double sigma) {
    
  double U1, U2, W, mult;
  static double X1, X2;
  static int call = 0;
 
  if (call == 1) {
      
      call = !call;
      return (mu + sigma * (double) X2);
    }
 
  do {
      
      U1 = -1 + ((double) rand () / RAND_MAX) * 2;
      U2 = -1 + ((double) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    } while (W >= 1 || W == 0);
 
  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;
 
  call = !call;
 
  return (mu + sigma * (double) X1);
}

//This calculates a new location for the robot, in the robot's current frame of reference, along a Levy distribution
//It returns a struct Vector which is just an {x,y} position.
Vector levyRandn(double mu){

  double U1, U2, U3, phi, r, mu_minusone, mu_twominus, mu_minusone_inverted, x, y;
  mu_minusone = mu - 1;
  mu_twominus = 2 - mu;
  mu_minusone_inverted = 1 / mu_minusone;
  //Generate 3 random numbers
  U1 = -1 + ((double) rand () / RAND_MAX) * 2;
  U2 = -1 + ((double) rand () / RAND_MAX) * 2;
  U3 = -1 + ((double) rand () / RAND_MAX) * 2;
  U1 = U1*(M_PI/2);
  U2 = (U2+1)/2;
  r = (sin(mu_minusone * U1) / pow(cos(U1), mu_minusone_inverted) ) * pow((cos(mu_twominus * U1) / U2), (mu_twominus / mu_minusone));
  phi = U3 * M_PI;
  x = r * cos(phi);
  y = r * sin(phi);
  Vector result = {x,y};
  return result;
}


//This function wrapes all the others which calculate aspects of the Levy Motion
void calculateLevyMotion() {

   //Calculate new postion using Levy   
   Vector newPosition = levyRandn(global_mu);
   //calculate the distance from current position
   distance_d = norm(&newPosition);
   angle_theta = rotation(&newPosition);
   //set the current_maneuver parameter to the current wheel encoder value
   current_maneuver = wb_position_sensor_get_value(left_position_sensor); 
   //calculate the wheel rotations to perform the turn
   total_units_rotation = (angle_theta * AXLE_LENGTH / WHEEL_DIAMETER); 
   //calcualte the differnce from current to goal movement
   goal_units_rotation = current_maneuver + total_units_rotation;
   //calculate the wheel rotations to perform the distance movement
   total_units_distance = distance_d / WHEEL_DIAMETER;
   //Move to next state in the state machine
   current_state++;
}

void rotateByAngle() {
   
   if (angle_theta > 0){
     wb_motor_set_velocity(left_motor, speed_unit * left_motor_speed);
     wb_motor_set_velocity(right_motor, - speed_unit * right_motor_speed);
   } else{
   
     wb_motor_set_velocity(left_motor, - speed_unit * left_motor_speed);
     wb_motor_set_velocity(right_motor, speed_unit * right_motor_speed);
   }
   current_maneuver = wb_position_sensor_get_value(left_position_sensor);
   
   bool stop1 = (current_maneuver < goal_units_rotation) && (angle_theta < 0);
   bool stop2 = (current_maneuver > goal_units_rotation) && (angle_theta > 0);

   if(stop1 || stop2){
     //stop moving
     wb_motor_set_velocity(left_motor, 0.0);
     wb_motor_set_velocity(right_motor, 0.0);

     current_maneuver = wb_position_sensor_get_value(left_position_sensor);
     goal_units_distance = current_maneuver + total_units_distance;
     //change to next state (moving forward)
     current_state++;
   }
}

void moveForwardByParameter() {

  wb_motor_set_velocity(left_motor, speed_unit * left_motor_speed);
  wb_motor_set_velocity(right_motor, speed_unit * right_motor_speed);

  // current_maneuver = abs(wb_position_sensor_get_value(left_position_sensor)-left_encoder_offset);
  current_maneuver = wb_position_sensor_get_value(left_position_sensor);

  if(current_maneuver > goal_units_distance){
    //stop moving
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    //change to first state
    current_state = 0;
  }
}


int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  //seeds the random number based on time
  //uncomment these two lines if running on real robot
  /* 
  int seed = time(NULL);
  srand ( time(NULL) );
  */
  //seeds the random number based on its PID
  //comment these two lines if running on real robot
  int seed = getpid();
  srand(seed);
  printf("My Seed number: %d\n", seed);

  // initialize devices
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  int iter = 0;
  for (iter = 0; iter < 8; iter++) {
    ps[iter] = wb_robot_get_device(ps_names[iter]);
    wb_distance_sensor_enable(ps[iter], TIME_STEP);
  }


  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  left_position_sensor = wb_robot_get_device("left wheel sensor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);


  //Calibrate the distance sensors
  calibrateIRSensors(DIST_SENSOR_CALIB_STEPS);



  //main loop

  while (wb_robot_step(TIME_STEP) != -1) {

    readProximitySensors(ps_values, ps);
    bool turnRight = leftObstacle();
    bool turnLeft = rightObstacle();

    if (turnRight) {
      // turn right
      wb_motor_set_velocity(left_motor, speed_unit * left_motor_speed);
      wb_motor_set_velocity(right_motor, speed_unit * - right_motor_speed);
      last_state = current_state;
      current_state = -1;
    }
    else if (turnLeft) {
      // turn left
      wb_motor_set_velocity(left_motor, speed_unit * - left_motor_speed);
      wb_motor_set_velocity(right_motor, speed_unit * right_motor_speed);
      last_state = current_state;
      current_state = -1;
    }
    
    switch(current_state) {

      case 0:
        calculateLevyMotion();
        break;

      case 1:
        // printf("Rotating!\n");
        rotateByAngle();
        break;

      case 2:
        // printf("Moving Forward!\n");
        moveForwardByParameter();
        break; 

      default : 
        current_state = last_state;
        if (current_state == 1){
          current_state = 2;
        }
    }
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  return 0;
}
