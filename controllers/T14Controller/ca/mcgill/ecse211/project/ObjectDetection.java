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


/**
 * Allows the robot to detect objects, avoid them and determine the difference between obstacles.
 */
public class ObjectDetection {

  // initializations
  private static HashMap<Double, Point> PointsList = new HashMap<Double, Point>();
  private static LinkedHashMap<Double, Integer> angleMap;
  private static ArrayList<Double> tempTorque = new ArrayList<Double>();
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
      if (detectObjInPath(objDist, DETECTION_THRESHOLD) && angle != prevAngle) {
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
   * Determines whether detected object is a block or not
   * @param objDist The ultrasonic sensor reading
   * @return is the object a block
   */
  public static boolean detectBlock(Integer objDist) {
    /*
     * if not in a certain threshold then the object is not a block
     */
    double THRESHOLD = 20;
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
   * Checks if an object has been detected within the threshold. This method throws out values if they
   * correspond to ramps, tunnels, bins
   * @param objDist The distance the ultrasonic sensor has read
   * @param threshold Distance where the object is considered to be detected
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

    //green ramp
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
   * Checks if an object has been detected within a 2 tile radius. This method throws out values if they
   * correspond to tunnels or walls
   * 
   * @param objDist The distance the ultrasonic sensor has read
   * @param threshold Distance where the object is considered to be detected
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

    // Detected a wall

    if (XF >= 15 * (TILE_SIZE) || XF <= 0 || (YF >= 8.5 * TILE_SIZE) || YF <= 0.2) {
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

  /**
   * Method which ensures that robot will not collide into obstacle throughout trajectory, will follow a following
   * algorithm and put back facing to original path once obstacle dealt with. This is specifically for avoidance outside
   * the search zone.
   * @param destination Point location
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
        if (Math.abs(currentAngle - startingAngle) > 55) {
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
      Driver.moveStraightFor(5* TILE_SIZE);
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
      while (!detectWallOrObject(readUsDistance(), DETECTION_THRESHOLD/2)) {
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

  /**
   * Method urges robot to detect and measure the torque while pushing every block in its search zone, all the while
   * avoiding potential objects and obstacles that may be found in its respective search zone
   */
  public static void ZoneDetection() {

    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

    double totalTorque = 0;
    double avgTorque = 0;
    Point szll;
    Point szur;

    // Find middle of the search zone based on starting zone
    if (STARTING_COLOR.equals("red")) {
      szll = szr.ll;
      szur = szr.ur;
    } else {
      szll = szg.ll;
      szur = szg.ur;
    }

    Point middleOfSZ = new Point((szll.x + szur.x) / 2, (szll.y + szur.y) / 2);
    // travel to the middlePoint, all the while avoiding obstacles in the search zone
    Navigation.turnTo(Navigation.getDestinationAngle(current, middleOfSZ));
    ObjectDetection.objectAvoider(middleOfSZ);

    // robot firstly finds all objects close to it in its search zone
    findObjects();


    // If it doesn't find objects have it sweep again
    if (angleMap.size() == 0) {
      Driver.turnBy(180);
      findObjects();
    }

    // check objects it findObjects(), store the angles they are located at with respect to robot in set
    Set<Double> angles = angleMap.keySet();

    // verify at each of these angle whether a block is present
    for (Double i : angles) {
      // turn to angle
      Navigation.turnTo(i);
      // if block is detected at angle, iteration stops, robot moves to block
      if (detectBlock(readUsDistance())) {
        Navigation.moveToBlock(Map.entry(i, readUsDistance()));
        // initial and current location stored
        Point initial = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
        // drivers travels distance of 1, average torque calculated throughout quick block push
        while (Navigation.distanceBetween(initial, current) < 1) {
          Driver.forward();
          tempTorque.add((leftMotor.getTorque() + rightMotor.getTorque()) / 2);
          current.x = odometer.getXyt()[0];
          current.y = odometer.getXyt()[1] / TILE_SIZE;
        }
        // find total torque of list, and then get average
        for (Double j : tempTorque) {
          totalTorque += j;
        }
        avgTorque = totalTorque / tempTorque.size();
        // store this average torque in hashmap, clear tempTorque for future use
        tempTorque.clear();
        PointsList.put(avgTorque, current);
      }
      // continue iterating
      continue;
    }
    // travel to the heaviest block (while avoiding obstacles) and get it in the bin
    objectAvoider(tree.lastEntry().getValue());
    printBlock(tree.lastEntry().getKey());
    Navigation.pushTo();
    Navigation.pushObjectOnRampAndReturn();

    // if time allows it dump blocks in bins in ascending order of weight to get as many blocks in as possible faslyr
    if (timer.getTime() > 60) {

      for (Point location : tree.values()) {
        objectAvoider(location);
        Navigation.pushTo();
        Navigation.pushObjectOnRampAndReturn();
      }
    }
  }

  /**
   * Sorts out HashMap to have the locations of the heaviest blocks stored in ascending order, prints out weight and
   * block of block at hand
   * @param avgTorque averageTorque of block
   */
  public static void printBlock(double avgTorque) {

    // store torques in arraylist in order establish in treemap
    ArrayList<Double> torques = new ArrayList<Double>(tree.keySet());
    double totTorque = 0;

    // store index of torque in list
    int index = torques.indexOf(avgTorque);

    // message to be printed prior to block being dumped into bin
    System.out.println("Container with weight:" + index + 1 + "identitfied.");
  }


  /**
   * 
   * @return the map with where all the objects have been detected with respect to the robot
   */
  public static LinkedHashMap<Double, Integer> getAngleMap() {
    return angleMap;
  }

  /**
   * Prints the values in the angleMap
   */
  public static void printMap() {
    // sortMapByValue();
    for (Map.Entry<Double, Integer> x : angleMap.entrySet()) {
      System.out.println("Angle " + x.getKey() + " Distance " + x.getValue());
    }
  }
   /** 
   * Simple method for calculating the mass of a block. 
   * Mass equation is taken from the torque characterization test. 
   */
  public static void PrintmassOfBlock() {
    Driver.forward();
    // Skip the first half tile since the torque is inconsistent
    sleepFor(30000);
    System.out.println("starting to calculate torque");
    double totaltorque=0;
    for (int i=1 ; i < 100 ; i++) {
      totaltorque += ((leftMotor.getTorque() + rightMotor.getTorque()) / 2);
      sleepFor(200);
    }
    Driver.stopMotors();
    System.out.println("The mass of the block is " + (totaltorque/100 + 0.0457)/0.1777 + " kg");
  }
}
