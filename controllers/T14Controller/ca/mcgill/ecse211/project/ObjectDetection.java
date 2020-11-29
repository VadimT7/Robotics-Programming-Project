package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.readUsDistance;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;
import ca.mcgill.ecse211.playingfield.Point;



public class ObjectDetection {

  // initializations
  private static HashMap<Double, Point> PointsList = new HashMap<Double, Point>();
  private static LinkedHashMap<Double, Integer> angleMap;
  private static ArrayList<Double> allTorque = new ArrayList<Double>();
  private static boolean isBlock;
  private static double[] prevAngles = new double[2];
  // treemap sorts heaviest blocks in ascending order based on keys
  private static TreeMap<Double, Point> tree = new TreeMap<Double, Point>(PointsList);

  /**
   * Saves any objects that is a block into a hashmap
   * 
   * @return Hashmap with the angle and distance of a block
   */
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

      // Throw out objects over 2 tile distances away/ at the same angle
      if (detectObjInPath(objDist, DETECTION_THRESHOLD / 2 + 15) && angle != prevAngle) {
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
    double THRESHOLD = 25;
    double noise = 5;

    // Save the angle
    double angle1 = (usMotor.getTachoCount() + 360) % 360;

    // Rotate opposite direction to find the other edge
    usMotor.setSpeed(ROTATE_SPEED / 4);
    usMotor.forward();
    while ((objDist <= (DETECTION_THRESHOLD + noise) && usMotor.getTachoCount() < 90)) {
      objDist = readUsDistance();
    }
    usMotor.stop();

    // Save the second angle
    double angle2 = (usMotor.getTachoCount() + 360) % 360;

    // Throw out value if this second angle was the same as the previous
    if (prevAngles[1] == angle2) {
      return false;
    }

    System.out.println(angle1 + " angle 2 " + angle2);

    // Save the angles
    prevAngles[0] = angle1;
    prevAngles[1] = angle2;

    // Verify that the width is under a certain threshold
    double angleDiff = Math.abs(Navigation.minimalAngle(angle1, angle2));
    if (angleDiff > THRESHOLD || angleDiff < 5) {
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
  public static boolean detectObjInPath(double objDist, double threshold) {

    // Object out of detection range
    if (objDist > threshold) {
      return false;
    }

    // Retrieve the current position and angle
    double X = odometer.getXyt()[0];
    double Y = odometer.getXyt()[1];
    double angle = Math.toRadians((odometer.getXyt()[2] + usMotor.getTachoCount()));

    // Calculate final point coordinates based on distance
    double XF = X + Math.sin(angle) * objDist / 100.0;
    double YF = Y + Math.cos(angle) * objDist / 100.0;
    angle = Math.toDegrees(angle);

    // Detected a wall
    if (XF >= 15 * (TILE_SIZE) || XF <= 0 || (YF >= 8.5 * TILE_SIZE) || YF <= 0.3) {
      return false;
    }

    // Check if any points are bin/tunnel coordinates, these are false positives
    // Check if it matches the red tunnel
    if (XF >= tnr.ll.x * (TILE_SIZE) && XF <= tnr.ur.x * (TILE_SIZE) && YF >= tnr.ll.y * (TILE_SIZE)
        && YF <= tnr.ur.y * (TILE_SIZE)) {
      return false;
    }

    // Check if it matches the green tunnel
    if (XF >= tng.ll.x * (TILE_SIZE) && XF <= tng.ur.x * (TILE_SIZE) && YF >= tng.ll.y * (TILE_SIZE)
        && YF <= tng.ur.y * (TILE_SIZE)) {
      return false;
    }

    // Check if coordinates are in the bin
    double maxY;
    double maxX;
    double minY;
    double minX;


    // Red ramp
    if (rr.left.x > rr.right.x) {
      maxY = rr.left.y - 2;
      minY = rr.left.y;
      maxX = rr.left.x;
      minX = rr.right.x;
    } else if (rr.left.x < rr.right.x) {
      maxY = rr.left.y + 2;
      minY = rr.left.y;
      maxX = rr.right.x;
      minX = rr.left.x;
    } else if (rr.left.y < rr.right.y) {
      maxY = rr.right.y;
      minY = rr.left.y;
      maxX = rr.left.x - 2;
      minX = rr.right.x;
    } else {
      maxY = rr.left.y;
      minY = rr.right.y;
      maxX = rr.left.x + 2;
      minX = rr.right.x;
    }

    // Check if it matches the green ramp
    if (XF >= minX * (TILE_SIZE) && XF <= maxX * (TILE_SIZE) && YF >= minY * (TILE_SIZE) && YF <= maxY * (TILE_SIZE)) {
      return false;
    }

    if (gr.left.x > gr.right.x) {
      maxY = gr.left.y - 2;
      minY = gr.left.y;
      maxX = gr.left.x;
      minX = gr.right.x;
    } else if (gr.left.x < gr.right.x) {
      maxY = gr.left.y + 2;
      minY = gr.left.y;
      maxX = gr.right.x;
      minX = gr.left.x;
    } else if (gr.left.y < gr.right.y) {
      maxY = gr.right.y;
      minY = gr.left.y;
      maxX = gr.left.x - 2;
      minX = gr.right.x;
    } else {
      maxY = gr.left.y;
      minY = gr.right.y;
      maxX = gr.left.x + 2;
      minX = gr.right.x;
    }

    // Check if it matches the green ramp
    if (XF >= minX * (TILE_SIZE) && XF <= maxX * (TILE_SIZE) && YF >= minY * (TILE_SIZE) && YF <= maxY * (TILE_SIZE)) {
      return false;
    }

    return true;
  }


  /**
   * 
   * @param objDist
   * @param threshold
   * @return if an object that is not a wall has been detected
   */
  public static boolean detectWallOrObject(double objDist, double threshold) {

    // Object out of detection range
    if (objDist > threshold) {
      return false;
    }

    // Retrieve the current position and angle
    double X = odometer.getXyt()[0];
    double Y = odometer.getXyt()[1];
    double angle = Math.toRadians((odometer.getXyt()[2] + usMotor.getTachoCount()));

    // Calculate final point coordinates based on distance
    double XF = X + Math.sin(angle) * objDist / 100.0;
    double YF = Y + Math.cos(angle) * objDist / 100.0;
    angle = Math.toDegrees(angle);

    System.out.println(XF + " " + YF);

    // Detected a wall
    if (XF >= 15 * (TILE_SIZE) || XF <= 0 || (YF >= 9.5 * TILE_SIZE) || YF <= 0.2) {
      return false;
    }

    // Check if any points are bin/tunnel coordinates, these are false positives
    // Check if it matches the red tunnel
    if (XF >= tnr.ll.x * (TILE_SIZE) && XF <= tnr.ur.x * (TILE_SIZE) && YF >= tnr.ll.y * (TILE_SIZE)
        && YF <= tnr.ur.y * (TILE_SIZE)) {
      return false;
    }

    // Check if it matches the green tunnel
    if (XF >= tng.ll.x * (TILE_SIZE) && XF <= tng.ur.x * (TILE_SIZE) && YF >= tng.ll.y * (TILE_SIZE)
        && YF <= tng.ur.y * (TILE_SIZE)) {
      return false;
    }
    return true;
  }


  /*
   * Method which ensures that robot will not collide into obstacle throughout trajectory, will follow a following
   * algorithm and put back facing to original path once obstacle dealt with. This is specifically for avoidance outside
   * the search zone.
   */
  public static void objectAvoider(Point destination) {

    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

    if (usMotor.getTachoCount() != -15) {
      usMotor.setSpeed(ROTATE_SPEED);
      usMotor.rotate(-15 - usMotor.getTachoCount(), false);
    }

    if (detectWallOrObject(readUsDistance(), TILE_SIZE * 100)) {
      // while robot is still in threshold vicinity of object, it backs up and turns to 90 degrees from its initial
      // angle
      Driver.setSpeed(ROTATE_SPEED);

      double startingAngle = odometer.getXyt()[2];
      double currentAngle = startingAngle;
      Driver.rotateClk();

      while ((detectWallOrObject(readUsDistance(), 2 * DETECTION_THRESHOLD))) {
        if (Math.abs(currentAngle - startingAngle) > 45) {
          Driver.rotateCClk();
        }
        currentAngle = odometer.getXyt()[2];
      }

      // Turn an extra 20 to clear the object
      if (startingAngle - currentAngle > 0) {
        Driver.turnBy(-20);
      } else {
        Driver.turnBy(20);
      }

      // move straight
      Driver.moveStraightFor(1.5);
      // repeat process to get to destination
      objectAvoider(destination);
    }

    // if small distance between current point and destination, just move straight
    else if (Navigation.distanceBetween(current, destination) < 1.5) {
      Navigation.travelTo(destination);
      usMotor.rotate(15, false);
    }
    // if object is outside given range of a tile and the robot is too far from its destination
    else {
      double destinationTheta = Navigation.getDestinationAngle(current, destination);
      Navigation.turnTo(destinationTheta);
      // move straight towards destination while this is still the case
      while (!detectWallOrObject(readUsDistance(), DETECTION_THRESHOLD)) {
        // Update the position
        current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
        // record angle between current and destination points
        Driver.forward();

        if (Navigation.distanceBetween(current, destination) <= 1.5) {
          break;
        }
        // do not exceed island bounds
        if (current.y <= island.ll.y + 0.4) {
          Driver.stopMotors();
          Navigation.turnTo(90);
          break;
        }


      }
      // when while loop breaks because object is read, call method again.
      objectAvoider(destination);
    }
  }

  /*
   * Method urges robot to detect and measure the torque while pushing every block in its search zone, all the while
   * avoiding potential objects and obstacles that may be found in its respective search zone
   */
  // public static void ZoneDetection() {

  // current location stored
  // Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

  // robot firstly finds all objects close to it in its search zone
  // findObjects();

  // check objects it findObjects(), store the angles they are located at with respect to robot in set
  // Set <Double> angles= angleMap.keySet();

  // verify at each of these angle whether a block is present
  // for (Double i : angles) {
  // turn to angle
  // Navigation.turnTo(i);
  // if block is detected at angle, itertion stops
  // if(detectBlock(readUsDistance())) {
  // break;
  // }
  // continue;
  // }
  // move straight to the block
  // Driver.moveStraightFor(readUsDistance()/100);

  // }

  /**
   * Sorts out HashMap to have the locations of the heaviest blocks stored in ascending order, prints out weight and
   * block of block at hand
   */
  public static void printBlock() {

    // store torques in arraylist in order establish in treemap
    ArrayList<Double> torques = new ArrayList<Double>(tree.keySet());
    double totTorque = 0;

    // calculate average torque of torques currently stored in arraylist
    for (Double t : allTorque) {
      totTorque += t;
    }

    double avgTorque = totTorque / allTorque.size();

    // store index of torque in list
    int index = torques.indexOf(avgTorque);

    System.out.println("Container with weight:" + index + 1 + "identitfied.");

  }

  public static void rotateOutOfObject() {
    Driver.setSpeed(ROTATE_SPEED);

    double startingAngle = odometer.getXyt()[2];
    double currentAngle = startingAngle;
    Driver.rotateCClk();

    while (readUsDistance() < TILE_SIZE * 100) {
      currentAngle = odometer.getXyt()[2];
      if (Math.abs(currentAngle - startingAngle) > 90) {
        Driver.rotateClk();
      }
    }
  }


  public static LinkedHashMap<Double, Integer> getAngleMap() {
    return angleMap;
  }

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
