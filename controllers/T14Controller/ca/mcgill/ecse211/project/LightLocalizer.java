package ca.mcgill.ecse211.project;

// importing necessary libraries
import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.playingfield.Region;
import simlejos.hardware.ev3.LocalEV3;
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
  private static float X = 70;

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
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-COLOR_SENSOR_TO_WHEEL_DIST/TILE_SIZE);
    // when reached, turn 90 degrees away from the wall with set rotation speed.
    Driver.setSpeed(ROTATE_SPEED);

    double turningAngle = 90.0;
    if ((x == TILE_SIZE && angle == 180) || (x == 14 * TILE_SIZE && angle == 0)) {
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
    Driver.moveStraightFor(-COLOR_SENSOR_TO_WHEEL_DIST/TILE_SIZE);


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
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-COLOR_SENSOR_TO_WHEEL_DIST/TILE_SIZE);

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
    Driver.moveStraightFor(-COLOR_SENSOR_TO_WHEEL_DIST/TILE_SIZE);


    Driver.setSpeed(ROTATE_SPEED);
    // Have the robot move back half a tile to compensate for sensor positioning
    Driver.turnBy(-90);

    // simulation stops for good as robot reaches desired point
    Driver.stopMotors();
  }

  /** 
   * Travel in a straight line until the two back sensors are both on a line
   */
  public static void lineDetect() {
    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);

    while (!(lightBuffer[0] <= X) || !(lightBuffer2[0] <= X)) {
      Driver.setSpeed(FORWARD_SPEED);
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

  /**
   * Localize based on color and parameters given by the wifi server
   */
  public static void startLocalize() {
    STARTING_COLOR = detectStartingColor();
    Region startingRegion;
    double angle = 0;
    int corner = 0;
    double initialX = 1;
    double initialY = 1;
    
    //Determine starting corner based on parameters
    if (STARTING_COLOR.equals("red")) {
      // Use red corner parameters
      startingRegion = red;
      corner = redCorner;
    } else {
      // Use green corner parameters
      // UPPER RIGHT - 1 = X
      startingRegion = green;
      corner = greenCorner;
    }
    
    //Change y/x coordinates according to which corner the robot is located in
    if(corner == 3 || corner  == 2) {
      initialY = startingRegion.ur.y-1;
      angle = 180;
    }
    
    if(corner == 2 || corner == 1) {
      initialX = startingRegion.ur.x-1;
    }
    
    localize(initialX * TILE_SIZE, initialY * TILE_SIZE, angle);
  }

  public static String detectStartingColor() {
    colorSensor.fetchSample(lightBuffer, 0);
    colorSensor2.fetchSample(lightBuffer2, 0);
    if (lightBuffer[0] > 200 || lightBuffer2[0] > 200) {
      return "red";
    }
    return "green";
  }


}
