// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#include "encoders.h"
// Class to track robot position.
class Kinematics_c {
  public:
  
    // Constructor, must exist.
    Kinematics_c() {

    } 

    float count_wheel_left = 0;
    float count_wheel_right = 0;

    float count_wheel_left_temp = 0;
    float count_wheel_right_temp = 0;

    float count_difference_left = 0;
    float count_difference_right = 0;

    float count_difference_left_cum_mean = 0;
    float count_difference_right_cum_mean = 0;
    float n = 0;
    
    float x_global = 0;
    float y_global = 0;
    
    float distance_moved = 0;
    float displacement = 0;
    
    float x_global_debug = 0;
    float y_global_debug = 0;

    float lastTime = 0;
    float currentTime = 0;

    float interval = 0;
    float Xr = 0;
    float rotationalChange = 0;
    float currentRotation = 0;
    float currentRotationCutoff = 0;
    float movementMultiplier = 3;

    float hallRatio = 358.3;

    double pi = 3.14159265359;

    bool recordKinematics = true;
    
    // Use this function to update
    // your kinematics
    void resetKinematics(){
      count_wheel_left = 0;
      count_wheel_right = 0;

      count_wheel_left_temp = 0;
      count_wheel_right_temp = 0;

      count_difference_left = 0;
      count_difference_right = 0;

      x_global = 0;
      y_global = 0;

      x_global_debug = 0;
      y_global_debug = 0;

      lastTime = 0;
      currentTime = 0;

      interval = 0;
      Xr = 0;
      rotationalChange = 0;
      currentRotation = 0;
      currentRotationCutoff = 0;

      count_difference_left_cum_mean = 0;
      count_difference_right_cum_mean = 0;
      n = 0;

      distance_moved = 0;

      displacement = 0;
    }
    void enableKinematics(){
      recordKinematics = true;
    }

    
    void updateLoop() {
      n ++;
      // get current time
      currentTime =  micros();
      interval = currentTime - lastTime;
      interval /= 1000000;

      // firstly, poll the encoders for the volatile rotation count values

      
      count_wheel_left_temp = count_e0;
      count_wheel_right_temp = count_e1;

      // correct abnormalities

      if(count_wheel_right_temp == count_wheel_left_temp + 1.0 || count_wheel_right_temp == count_wheel_left_temp + 2.0) count_wheel_right_temp = count_wheel_left_temp;
      else if(count_wheel_left_temp == count_wheel_right_temp + 1.0 || count_wheel_left_temp == count_wheel_right_temp + 2.0) count_wheel_left_temp = count_wheel_right_temp;

      
      
      // get difference in values compared to last saved ones
      count_difference_left = count_wheel_left_temp - count_wheel_left;
      count_difference_right = count_wheel_right_temp - count_wheel_right;
      count_difference_left*=-1;
      count_difference_right*=-1;
      // save new rotation values
      if(recordKinematics){
        count_wheel_right = count_wheel_right_temp;
        count_wheel_left = count_wheel_left_temp;
      }
      count_difference_left_cum_mean += count_wheel_left;
      count_difference_left_cum_mean /= n;

      count_difference_right_cum_mean += count_wheel_right;
      count_difference_right_cum_mean /= n;



      double wheelRotationalAmt = -(((double)count_difference_left - (double)count_difference_right)/(hallRatio)) * 6.28318530718;
      double wheelForwardAmt = ((double)(count_difference_left + count_difference_right)/(hallRatio));
      // Xr = ((wheel radius * rotation velocity left) /2) + ((wheel radius * rotation velocity right) /2) * interval
      Xr = (wheelForwardAmt * 0.034)/2.0;
      distance_moved += Xr * movementMultiplier;
      
      // Yr = 0
      // L = distance between wheel and midpoint of bot
      // 0r (rotationalChange) = ((wheel radius * rotation velocity left) /2*L) + ((wheel radius * rotation velocity right) /2*L)
      //rotationalChange = ((0.017 * (count_difference_left / interval)) / (2.0* 0.045)) - ((0.017 * (count_difference_right / (interval))) / (2.0* 0.045));
      
      
      float robotRotationalAmt = (0.017 * wheelRotationalAmt) / 0.09; 

      //robotRotationalVelocity /= 3.14159265359;

      rotationalChange = robotRotationalAmt;
     // rotationalChange = (0.017 * ((((count_difference_left - count_difference_right)/(12*30))) / interval)) / (2.0* 0.045);
      
      // define new rotation
      // defined as last rotaion in radians, plus rotational change
      // 0i = 0i + 0r
      currentRotation += rotationalChange;
      currentRotationCutoff += rotationalChange;
      if(currentRotationCutoff > 6.28318530718){
        currentRotationCutoff = currentRotationCutoff - 6.28318530718;
      }
      else if (currentRotationCutoff < -6.28318530718){
        currentRotationCutoff = currentRotationCutoff + 6.28318530718;
      }
      // convert local motion to global motion
      
      // Xdelta = Xr * cos(0i)
      // Ydelta = Xr * sin(0i)

      double Xdelta = Xr * cos(currentRotation) * movementMultiplier;
      double Ydelta = Xr * sin(currentRotation) * movementMultiplier;
      // update rotation matrix

      // apply new displacement by:
      /*
       * X(t+1) = X(t) + Xdelta
       * Y(t+1) = Y(t) + Ydelta
       */
       if(recordKinematics){
         x_global += Xdelta;
         y_global += Ydelta;
       }

       displacement = sqrt(x_global*x_global + y_global*y_global); 
       
       x_global_debug += Xdelta;
       y_global_debug += Ydelta;
       
//       Serial.println((String) "displacement is  " + displacement);
//       Serial.println((String) "wheelForwardAmt is  " + wheelForwardAmt);
//       Serial.println(Xr, 8);
//       Serial.println((String) "count_difference_left is " + count_difference_left);
//       Serial.println((String) "count_difference_right is " + count_difference_right);
//       Serial.println((String) "count_difference_left_cum_mean is " + count_difference_left_cum_mean);
//       Serial.println((String) "count_difference_right_cum_mean is " + count_difference_right_cum_mean);
//       Serial.println((String) "rotationalChange is " + rotationalChange);
//       Serial.println((String) "rotation is " + currentRotation + " which is " + currentRotation * 57.2958);
//       
//       Serial.println((String) "deltas are " + Xdelta + "," + Ydelta);
//       Serial.println((String) "coordinates are " + x_global + "," + y_global);
//       Serial.println("=================================================");
       
       lastTime = currentTime;
    }

};



#endif
