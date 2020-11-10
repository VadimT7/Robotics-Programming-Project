package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.readUsDistance;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import ca.mcgill.ecse211.playingfield.Point;



public class ObjectDetection {

  public static final double TWO_TILE_DIST = 60.96;
  private static HashMap<Double, Integer> angleMap;

  public static HashMap<Double, Integer> findObjects() {
    angleMap = new HashMap<>();
    // TODO wrap the whole method in a loop while the robot is rotating
    // usMotor.rotate(360, true);

    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.backward();
    int startTacho = 0;
    while (startTacho > -90) {
      double angle = -usMotor.getTachoCount();
      int objDist = readUsDistance();
      // Throw out objects over 2 tile distances away
      if (detectObjInPath()) {
        angleMap.put(angle, objDist);
      //  break;
      }

      // Get the current tachocount
      startTacho = usMotor.getTachoCount();
    }
    usMotor.rotate(90, false);
    usMotor.resetTachoCount();
    usMotor.forward();
    while (startTacho < 90) {
      double angle = usMotor.getTachoCount();
      
      angle = 360 - angle;
      
      int objDist = readUsDistance();
      // Throw out objects over 2 tile distances away
      if (detectObjInPath()) {
        angleMap.put(angle, objDist);
      }

      // Get the current tachocount
      startTacho = usMotor.getTachoCount();
    }


    usMotor.stop();
    usMotor.resetTachoCount();
    return angleMap;
  }

  /**
   * 
   * @return is the object a block
   */
  public static boolean detectBlock() {


    /*
     * if not in a certain threshold then the object is not a block
     */
    int objDist = readUsDistance();

    double THRESHOLD = 20;

    // TODO ROTATE TO FIND EDGE
    double angle1 = odometer.getXyt()[2];

    // TODO ROTATE OPPOSITED DIRECTION TO FIND OTHER EDGE
    double angle2 = odometer.getXyt()[2];

    if (360 - angle1 - angle2 > THRESHOLD) {
      return false;
    }

    return true;
  }


  public static boolean detectObjInPath() {

    int objDist = readUsDistance();
    // System.out.println(objDist);
    double X = odometer.getXyt()[0];
    double Y = odometer.getXyt()[1];
    double angle = odometer.getXyt()[2];

    // Throw out false positives (eg: walls, bins, tunnel)
    // TODO determine the bounds of the map

    // Calculate final point coordinates based on distance
    double XF = X + Math.sin(angle) * objDist;
    double YF = Y + Math.cos(angle) * objDist;

    //System.out.println("XF " + XF + " YF " + YF);

    /*
     * ========= ALL CASES WHERE THE READ VALUE SHOULD BE DISCARDED =========
     * 
     * if(xf > bounds || yf > bounds )
     * 
     * 
     * if(xf < bounds || yf < bounds )
     * 
     * round xf and yf
     * 
     * Point p = new Point(XF/TILE_SIZE, YF/TILE_SIZE)
     * 
     * if( point == tunnelCoordinates)
     * 
     * if( point == binCoordinates)
     * 
     * 
     */

    // Detected a wall
    if (XF >= 15 * (TILE_SIZE * 100) || XF <= 0 || (YF >= 9 * TILE_SIZE * 100) || YF <= 0) {
      return false;
    }

    // Check if any points are bin/tunnel coordinates
    Point p = new Point(Math.round(XF), Math.round(YF));



    // Object out of detection range
    if (objDist > DETECTION_THRESHOLD) {
      return false;
    }
    return true;
  }

  // Print values
  public static void printMap() {
    //sortMapByValue();
    for (Map.Entry<Double, Integer> x : angleMap.entrySet()) {
      System.out.println("Angle " + x.getKey() + " Distance " + x.getValue());
    }
  }

  // Sort the hashmap by its values
  public static HashMap<Double, Integer> sortMapByValue() {
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
