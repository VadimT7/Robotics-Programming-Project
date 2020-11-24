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
import ca.mcgill.ecse211.playingfield.Point;



public class ObjectDetection {

  private static LinkedHashMap<Double, Integer> angleMap;
  private static boolean isBlock;
  private static double[] prevAngles = new double[2];

  /**
   * Saves any objects that is a block into a hashmap
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

    //Start the 180 degree sweep
    while (startTacho < 90) {
      double angle = (odometer.getXyt()[2] + usMotor.getTachoCount() + 360) % 360;
      Integer objDist = readUsDistance();
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
   * Determine the width of a detected object to confirm if the detected object is a block
   * @param objDist the distance at which the block was detected
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
    usMotor.setSpeed(ROTATE_SPEED/4);
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
    
    //Save the angles
    prevAngles[0] = angle1;
    prevAngles[1] = angle2;

    // Verify that the width is under a certain threshold
    double angleDiff = Math.abs(angle1 - angle2);
    if (angleDiff > THRESHOLD || angleDiff < 5) {
      return false;
    }
    return true;
  }

  /**
   * Method checks if an object has been detected within a 2 tile radius. This method throws out values if they
   * correspond to ramps, tunnels, bins and walls
   * 
   * @param objDist the distance at which the object was detected
   * @return if an object has been detected
   */
  public static boolean detectObjInPath(int objDist) {

    // Object out of detection range
    if (objDist > DETECTION_THRESHOLD) {
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
    
    System.out.println(YF/TILE_SIZE);
    // Detected a wall
    if (XF >= 15 * (TILE_SIZE) || XF <= 0 || (YF >= 9 * TILE_SIZE) || YF <= 0) {
      return false;
    }

    // Check if any points are bin/tunnel coordinates, these are false positives
    // Check if it matches the red tunnel
    if (XF >= tnr.ll.x && XF <= tnr.ur.x && YF >= tnr.ll.y && YF <= tnr.ur.y) {
      return false;
    }

    // Check if it matches the green tunnel
    if (XF >= tng.ll.x && XF <= tng.ur.x && YF >= tng.ll.y && YF <= tng.ur.y) {
      return false;
    }
    
    return true;
  }

  /*
   * Method which ensures that robot will not collide into obstacle throughout trajectory, will follow a following
   * algorithm and put back facing to original path once obstacle dealt with. This is specifically for avoidance outside
   * the search zone.
   */
  public static void OutobjectAvoider(Point destination) {
    Point p1 = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    int objDist = readUsDistance();

    Point bounds = island.ll;

    System.out.println(objDist);
    if (readUsDistance() < 40) {
      Driver.moveStraightFor(0.2* -TILE_SIZE);
      // Rotate until we stop detecting it
      while (readUsDistance() < 40) {
        if (p1.y <= bounds.y + 0.4) {
          Driver.turnBy(-35);
        } else {
          Driver.turnBy(35);
        }
      }

      while (readUsDistance() > 45) {
        Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
        Driver.setSpeed(FORWARD_SPEED);
        Driver.forward();
        if (current.y <= bounds.y + 0.4) {
          Driver.stopMotors();
          Navigation.turnTo(90);
          break;
        }
      }
      Driver.stopMotors();
      OutobjectAvoider(destination);
    } else if (Navigation.distanceBetween(p1, destination) < 1) {
      Navigation.travelTo(destination);
    } else {
      int tempdist = readUsDistance();
      while (tempdist > 40) {
        System.out.println(tempdist);
        Driver.setSpeed(FORWARD_SPEED);
        Driver.forward();
        tempdist = readUsDistance();
      }
      Driver.stopMotors();
      OutobjectAvoider(destination);
    }



    // // trivial rotation angle, determine through testing
    // double rotAngle = 30;
    //
    // // trajectory to search zone commences
    // Navigation.travelToSearchZone();
    //
    // // robot behaviour should the robot detect an object (including blocks) throughout navigation
    // if (detectObjInPath(readUsDistance())) {
    //
    // // distance object will move by once object no longer detected
    // int distToObj = readUsDistance();
    // int initDist = distToObj;
    //
    // while (distToObj <= DETECTION_THRESHOLD) {
    // // robot rotates by arbitrary angle deemed fit through testing
    // Driver.turnBy(rotAngle);
    // // if object is no longer detected, proceed
    // if (!detectObjInPath(readUsDistance())) {
    //
    // // robot moves straight to bypass object
    // leftMotor.rotate(Driver.convertDistance(initDist* TILE_SIZE), true);
    // rightMotor.rotate(Driver.convertDistance(initDist * TILE_SIZE), true);
    // // intermediate point noted
    // Point initial = new Point(odometer.getXyt()[0], odometer.getXyt()[2]);
    //
    // Navigation.travelToSearchZone();
    //
    // // current point continuously updated throughout trajectory
    // Point current = new Point(odometer.getXyt()[1], odometer.getXyt()[2]);
    // // for every two tiles, conduct object detection
    // if (Navigation.distanceBetween(initial, current) == DETECTION_THRESHOLD) {
    // detectObjInPath(readUsDistance());
    // }
    // distToObj = readUsDistance();
    // }
    //
    // // if object still detected, rotate another 30 degrees.
    // else {
    //
    // Driver.turnBy(rotAngle);
    // }
    // System.out.println(distToObj);
    // }
    // Driver.stopMotors();
    // }

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
