package ca.mcgill.ecse211.project;

// importing necessary libraries
import static ca.mcgill.ecse211.project.Resources.*;
import simlejos.robotics.SampleProvider;

/**
 * Light localization class (for 2nd part of trajectory)
 */
public class LightLocalizer {

  private static SampleProvider colorSensor = leftColorSensor.getRedMode();
  private static SampleProvider colorSensor2 = rightColorSensor.getRedMode();
  // Initializations
  /** buffers for light sensors setup **/
  static float[] lightBuffer = new float[colorSensor.sampleSize()];
  static float[] lightBuffer2 = new float[colorSensor2.sampleSize()];
  /* Arbitrary boundary condition (choice was made through testing) */
  private static float X = 70;// 35.0f;

  /**
   * performs localization with the steps seen in video tutorial+slides **\
   */
  public static void localize(double x, double y, double angle) {
    /* initial readings of light sensor at very end of ultrasonic/beginning of light localization */
    // fetching values and storing them in empty arrays

    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);

    // moving forward while both sensors are not on the line
    while (!(lightBuffer[0] <= X) || !(lightBuffer2[0] <= X)) {
      Driver.setSpeed(FORWARD_SPEED);
      Driver.setAcceleration(ACCELERATION);

      // Stop the corresponding motor once it reaches a line
      if (lightBuffer[0] <= X) {
        leftMotor.stop();
      } else {
        leftMotor.forward();
      }

      if (lightBuffer2[0] <= X) {
        rightMotor.stop();
      } else {
        rightMotor.forward();
      }
      colorSensor.fetchSample(lightBuffer, 0);
      colorSensor2.fetchSample(lightBuffer2, 0);
      // System.out.println(lightBuffer[0]);
    }
    Driver.stopMotors();
    int rotate = Driver.convertDistance(COLOR_SENSOR_TO_WHEEL_DIST);
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-rotate);

    // when reached, turn 90 degrees away from the wall with set rotation speed.
    Driver.setSpeed(ROTATE_SPEED);
    
    double turningAngle = 90.0;
    if((x == TILE_SIZE && angle == 180) || (x == 14 * TILE_SIZE && angle == 0)) {
      turningAngle = -turningAngle;
    }
    
    Driver.turnBy(turningAngle);

    /* updating current reading and storing in arrays */

    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);

    while (!(lightBuffer[0] <= X) || !(lightBuffer2[0] <= X)) {
      Driver.setSpeed(FORWARD_SPEED);
      Driver.setAcceleration(ACCELERATION);

      // Stop the corresponding motor once it reaches a line
      if (lightBuffer[0] <= X) {
        leftMotor.stop();
      } else {
        leftMotor.forward();
      }

      if (lightBuffer2[0] <= X) {
        rightMotor.stop();
      } else {
        rightMotor.forward();
      }
      colorSensor.fetchSample(lightBuffer, 0);
      colorSensor2.fetchSample(lightBuffer2, 0);
    }
    Driver.stopMotors();
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-rotate);


    Driver.setSpeed(ROTATE_SPEED);
    // Have the robot move back half a tile to compensate for sensor positioning
    

    Driver.turnBy(-turningAngle);

    // simulation stops for good as robot reaches desired point
    Driver.stopMotors();
    odometer.setXyt(x, y, angle);
  }

  /**
   * Localize without setting any odometer parameters
   */
  public static void localize() {

    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);

    // moving forward while both sensors are not on the line
    while (!(lightBuffer[0] <= X) || !(lightBuffer2[0] <= X)) {
      Driver.setSpeed(FORWARD_SPEED);
      Driver.setAcceleration(ACCELERATION);

      // Stop the corresponding motor once it reaches a line
      if (lightBuffer[0] <= X) {
        leftMotor.stop();
      } else {
        leftMotor.forward();
      }

      if (lightBuffer2[0] <= X) {
        rightMotor.stop();
      } else {
        rightMotor.forward();
      }
      colorSensor.fetchSample(lightBuffer, 0);
      colorSensor2.fetchSample(lightBuffer2, 0);
      // System.out.println(lightBuffer[0]);
    }
    Driver.stopMotors();
    int rotate = Driver.convertDistance(COLOR_SENSOR_TO_WHEEL_DIST);
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-rotate);

    // when reached, turn 90 degrees away from the wall with set rotation speed.
    Driver.setSpeed(ROTATE_SPEED);
    Driver.turnBy(90.0);

    /* updating current reading and storing in arrays */

    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);

    while (!(lightBuffer[0] <= X) || !(lightBuffer2[0] <= X)) {
      Driver.setSpeed(FORWARD_SPEED);
      Driver.setAcceleration(ACCELERATION);

      // Stop the corresponding motor once it reaches a line
      if (lightBuffer[0] <= X) {
        leftMotor.stop();
      } else {
        leftMotor.forward();
      }

      if (lightBuffer2[0] <= X) {
        rightMotor.stop();
      } else {
        rightMotor.forward();
      }
      colorSensor.fetchSample(lightBuffer, 0);
      colorSensor2.fetchSample(lightBuffer2, 0);
    }
    Driver.stopMotors();
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-rotate);


    Driver.setSpeed(ROTATE_SPEED);
    // Have the robot move back half a tile to compensate for sensor positioning
    Driver.turnBy(-90);

    // simulation stops for good as robot reaches desired point
    Driver.stopMotors();
  }

  public static void lineDetect() {
    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);

    while (!(lightBuffer[0] <= X) || !(lightBuffer2[0] <= X)) {
      Driver.setSpeed(FORWARD_SPEED/2);
      if (lightBuffer[0] <= X) {
        leftMotor.stop();
      } else {
        leftMotor.forward();
      }

      if (lightBuffer2[0] <= X) {
        rightMotor.stop();
      } else {
        rightMotor.forward();
      }
      colorSensor.fetchSample(lightBuffer, 0);
      colorSensor2.fetchSample(lightBuffer2, 0);
    }
    Driver.stopMotors();
  }

  public static void startLocalize() {
     STARTING_COLOR = detectStartingColor();
    
    //TODO implement logic to determine the starting localization coordinates
    //To implement we need to check if the parameters given by the server are at the bounds of x/y
    //If either bound is equal to Y max then the robot will localize towards 180
    //Otherwise localize towards 0
    
    //Hard coded for now
    if(STARTING_COLOR.equals("red")) {
      //Use parameters given by server
      // UPPER BOUND - 1, hard coded for now
      localize(1*TILE_SIZE, 8 * TILE_SIZE, 180);
      odometer.printPositionInTileLengths();
    }else {
      //Use green corner parameters
      //UPPER RIGHT - 1 = X
      localize(14*TILE_SIZE, 1 * TILE_SIZE, 0);
      odometer.printPositionInTileLengths();
    }
      
  }
  public static String detectStartingColor() {
    colorSensor.fetchSample(lightBuffer, 0);
    System.out.println(lightBuffer[0]);
    if (lightBuffer[0] > 200) {
      return "red";
    }
    return "green";
  }


}
