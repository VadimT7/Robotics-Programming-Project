package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

import static ca.mcgill.ecse211.project.UltrasonicLocalizer.readUsDistance;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import ca.mcgill.ecse211.playingfield.Point;



public class ObjectDetection {

  private static LinkedHashMap<Double, Integer> angleMap;
  private static boolean isBlock;
  private static double[] prevAngles = new double[2];

  public static LinkedHashMap<Double, Integer> findObjects() {
    angleMap = new LinkedHashMap<>();

    // Set random prevangle
    prevAngles[0] = 360;
    prevAngles[1] = 360;

    // Rotate to 90 degrees to begin the 180 degree sweep
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.rotate(-90, false);
    usMotor.forward();

    // Save the tacho count of the ultrasonic sensor motor
    double startTacho = usMotor.getTachoCount();


    // arbitrary prev angle to prevent from reading the same angle twice
    double prevAngle = -10;

    while (startTacho < 90) {
      double angle = (odometer.getXyt()[2] + usMotor.getTachoCount() + 360) % 360;
      Integer objDist = readUsDistance();
      // System.out.println(objDist);
      // Throw out objects over 2 tile distances away/ at the same angle
      if (detectObjInPath(objDist) && angle != prevAngle) {
        // Stop rotation and latch onto object, determine width

        usMotor.stop();
        isBlock = detectBlock(objDist);

        // If the object detected is a block then add it to the map
        if (isBlock) {
          angleMap.put(angle, objDist);
        }
        // Continue the rotation
        usMotor.setSpeed(ROTATE_SPEED);
        usMotor.forward();
      }

      // Save the previous angle
      prevAngle = angle;

      // Get the current tachocount
      startTacho = usMotor.getTachoCount();
    }

    // Stop the rotation
    usMotor.stop();
    // Have the motor rotate back to 0 degrees (where the robot is facing)
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.rotate(-90, false);
    usMotor.stop();
    usMotor.resetTachoCount();

    return angleMap;
  }

  /**
   * 
   * @return is the object a block
   */
  public static boolean detectBlock(Integer objDist) {
    /*
     * if not in a certain threshold then the object is not a block
     */
    double THRESHOLD = 20;

    double maxTreshold = objDist;


    // Rotate to find the edge of the object of the object
    usMotor.setSpeed(ROTATE_SPEED / 4);
    usMotor.backward();
    while (objDist <= DETECTION_THRESHOLD && usMotor.getTachoCount() > -90) {
      objDist = readUsDistance();
    }
    maxTreshold = objDist;
    usMotor.stop();

    // Save the angle
    double angle1 = (usMotor.getTachoCount() + 360) % 360;

    double tempTacho = usMotor.getTachoCount();
    // Rotate opposite direction to find the other edge
    usMotor.setSpeed(ROTATE_SPEED / 4);
    usMotor.forward();
    while (((objDist <= maxTreshold && usMotor.getTachoCount() < 90) || usMotor.getTachoCount() < tempTacho + 5)) {
      objDist = readUsDistance();
    }
    usMotor.stop();

    // Save the second angle
    double angle2 = (usMotor.getTachoCount() + 360) % 360;

    // Throw out value if this second angle was the same as the previous
    if (prevAngles[1] == angle2) {
      return false;
    }

    // System.out.println(angle1 + " angle 2 " + angle2);

    // Save the angles
    prevAngles[0] = angle1;
    prevAngles[1] = angle2;

    // Verify that the width is under a certain threshold
    double angleDiff = Math.abs(Navigation.minimalAngle(angle1, angle2));
    if (angleDiff > THRESHOLD || angleDiff < 10) {
      return false;

    }
    return true;
  }

  /**
   * Method checks if an object has been detected within a 2 tile radius. This method throws out values if they
   * correspond to ramps, tunnels, bins
   * 
   * @return if an object has been detected
   */
  public static boolean detectObjInPath(double objDist) {

    // Object out of detection range
    if (objDist > DETECTION_THRESHOLD) {
      return false;
    }

    while (DETECTION_THRESHOLD >= readUsDistance() && readUsDistance() >= objDist) {

      // Retrieve the current angle
      double angle = Math.toRadians((odometer.getXyt()[2] + usMotor.getTachoCount()));


      return true;

    }
    return false;
  }

  /*
   * Method which ensures that robot will not collide into obstacle throughout trajectory, will follow a following
   * algorithm and put back facing to original path once obstacle dealt with. This is specifically for avoidance outside
   * the search zone.
   */
  public static void OutobjectAvoider(Point destination) {

    // initializations
    int thetaF;
    double objDist = 0;
    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

    while (Driver.isNavigating()) {
      // detect objects continuously during robot's travel
      findObjects();
      // obtain distances of detected objects
      Set<Double> distances = angleMap.keySet();
      // iterate through distances
      for (Double distance : distances) {
        // if distance is within detection threshold and an object is detected within distance, stop robot
        if (distance < TILE_SIZE && detectObjInPath(distance)) {
          objDist = distance;
          Driver.stopMotors();
          System.out.println(distance);
          break;
        }
        continue;
      }
    }
    // detect object's angle
    thetaF = angleMap.get(objDist);
    System.out.println(thetaF);
    // make sure robot is facing object in its path
    Navigation.turnTo(thetaF);

    if (readUsDistance() < TILE_SIZE) {
      // while robot is still in threshold vicinity of object, it backs up and turns to 90 degrees from its initial
      // angle
      while (readUsDistance() < TILE_SIZE) {
        Driver.moveStraightFor(0.3 * -TILE_SIZE);
        Navigation.turnTo(odometer.getXyt()[2] - 90);
      }
      // move straight
      Driver.moveStraightFor(0.5 * TILE_SIZE);
      // turn back to to original angle
      Navigation.turnTo(odometer.getXyt()[2] + 90);
      // repeat process to get to destination
      OutobjectAvoider(destination);
    }

    // if small distance between current point and destination, just move straight
    else if (Navigation.distanceBetween(current, destination) < 1) {
      Navigation.travelTo(destination);
    }
    // if object is outside given range of a tile
    else if (readUsDistance() > TILE_SIZE) {
      // move straight towards destination while this is still the case
      while (readUsDistance() > TILE_SIZE) {
        // record angle between current and destination points
        var destinationTheta = Navigation.getDestinationAngle(current, destination);
        // turn to destination and move forward
        Driver.turnBy(Navigation.minimalAngle(odometer.getXyt()[2], destinationTheta));
        Driver.forward();
        // do not exceed island bounds
        if (current.y <= island.ll.y + 0.4) {
          Driver.stopMotors();
          Navigation.turnTo(90);
          break;
        }
      }
      //when while loop breaks because object is read, call method again.
      OutobjectAvoider(destination);
    }
  }
  // Point p1 = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
  // int objDist = readUsDistance();

  // Point bounds = island.ll;

  // System.out.println(objDist);
  // if (readUsDistance() < TILE_SIZE) {
  // Driver.moveStraightFor(0.2* -TILE_SIZE);
  // Rotate until we stop detecting it
  // while (readUsDistance() < TILE_SIZE) {
  // if (p1.y <= bounds.y + 0.4) {
  // Driver.turnBy(-35);
  // } else {
  // Driver.turnBy(35);
  // }
  // }

  // while (readUsDistance() > TILE_SIZE) {
  // Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
  // Driver.setSpeed(FORWARD_SPEED);
  // Driver.forward();
  // if (current.y <= bounds.y + 0.4) {
  // Driver.stopMotors();
  // Navigation.turnTo(90);
  // break;
  // }
  // }
  // Driver.stopMotors();
  // OutobjectAvoider(destination);
  // } else if (Navigation.distanceBetween(p1, destination) < 1) {
  // Navigation.travelTo(destination);
  // } else {
  // int tempdist = readUsDistance();
  // while (tempdist > TILE_SIZE) {
  // System.out.println(tempdist);
  // Driver.setSpeed(FORWARD_SPEED);
  // Driver.forward();
  // tempdist = readUsDistance();
  // }
  // Driver.stopMotors();
  // OutobjectAvoider(destination);
  // }
  // }


  // Print values
  public static void printMap() {
    // sortMapByValue();
    for (Map.Entry<Double, Integer> x : angleMap.entrySet()) {
      System.out.println("Angle " + x.getKey() + " Distance " + x.getValue());
    }
  }

  // Sort the hashmap by its values
  public static LinkedHashMap<Double, Integer> sortMapByValue() {
    List<Map.Entry<Double, Integer>> list = new LinkedList<Map.Entry<Double, Integer>>(angleMap.entrySet());

    Collections.sort(list, new Comparator<Map.Entry<Double, Integer>>() {

      @Override
      public int compare(Entry<Double, Integer> o1, Entry<Double, Integer> o2) {
        return o1.getValue().compareTo(o2.getValue());
      }
    });

    angleMap.clear();

    for (Map.Entry<Double, Integer> x : list) {
      angleMap.put(x.getKey(), x.getValue());
    }

    return angleMap;

  }
}
