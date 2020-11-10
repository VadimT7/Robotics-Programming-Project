package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.playingfield.Region;

/**
 *
 */
public class UltrasonicLocalizer {

  /** Arbitrary constant used to find the heading */
  private final static int DIST = 35;

  /** Noise margin */
  private final static int K = 2;

  /** Min angle between 2 walls */
  private static final double thetaMin = 30;

  // Instance/Class variables
  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;
  /** Is the robot facing a wall */
  private static boolean detected;
  /** Heading of the robot */
  private static double heading;


  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];

  /**
   * If facing the wall the method detects a rising edge and a falling edge and calculates the heading. When facing away
   * from the wall the same principle is applied except it detects 2 falling edges
   */
  public static void localize() {

    // Change heading calculation based on the corner of the robot

    STARTING_COLOR = LightLocalizer.detectStartingColor();
    Region startingRegion;

    if (STARTING_COLOR.equals("red")) {
      // Use red corner parameters
      // UPPER BOUND - 1, hard coded for now
      startingRegion = red;
    } else {
      // Use green corner parameters
      // UPPER RIGHT - 1 = X
      startingRegion = green;
    }

    double wallHeading = 315;
    double noWallHeading = 135;

    if ((startingRegion.ll.x == 0 && startingRegion.ur.y == 9)
        || (startingRegion.ll.y == 0 && startingRegion.ur.x == 15)) {
      wallHeading = 45;
      noWallHeading = 225;
    }

    double alpha;
    double beta;
    // Determine if we are facing a wall
    detected = readUsDistance() < (DIST + K);

    // Detect a rising edge and a falling edge if facing a wall
    if (detected) {
      Driver.rotateClk();
      // Find rising edge
      while (detected) {
        detected = readUsDistance() < (DIST + K);
      }
      Driver.stopMotors();

      // Save the first detected angle
      alpha = odometer.getXyt()[2];

      // Restart rotation
      Driver.rotateClk();
      // Find falling edge
      while (!detected || Math.abs(odometer.getXyt()[2] - alpha) < thetaMin) {
        detected = readUsDistance() <= (DIST - K);
      }
      Driver.stopMotors();

      // Save the second detected angle
      beta = odometer.getXyt()[2];
      // Caculate the heading
      heading = wallHeading - (odometer.getXyt()[2] - (alpha + beta) / 2);
    }
    // Otherwise detect two falling edges
    else {
      // Start Rotation
      Driver.rotateClk();

      // Find falling edge
      while (!detected) {
        detected = readUsDistance() <= (DIST - K);
      }
      Driver.stopMotors();
      // Save the first detected angle
      alpha = odometer.getXyt()[2];


      Driver.rotateCClk();
      // Find falling edge
      while (!detected || Math.abs(odometer.getXyt()[2] - alpha) < thetaMin) {
        detected = readUsDistance() <= (DIST - K);
      }
      Driver.stopMotors();
      beta = odometer.getXyt()[2];

      // Save the second detected angle
      heading = noWallHeading - (odometer.getXyt()[2] - (alpha + beta) / 2);
    }
    Driver.turnBy(heading);

    // Reset odometer
    odometer.setXyt(0, 0, 0);
  }

  /**
   * Returns the filtered distance between the US sensor and an obstacle in cm.
   * 
   * @Author method taken from lab1
   */
  public static int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, cast to int, and filter
    return filter((int) (usData[0] * 100.0));
  }


  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @Author method taken from lab1
   *
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  static int filter(int distance) {
    if (distance >= MAX_SENSOR_DIST && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < MAX_SENSOR_DIST) {
        invalidSampleCount = 0; // reset filter and remember the input distance.
      }

      if (prevDistance - distance > 150) {
        return prevDistance;
      }
      prevDistance = distance;
      return distance;
    }
  }

}
