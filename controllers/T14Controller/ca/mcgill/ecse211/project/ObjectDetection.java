package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.DETECTION_THRESHOLD;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.usMotor;
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

  public static LinkedHashMap<Double, Integer> findObjects() {
    angleMap = new LinkedHashMap<>();
    
    //Set random prevangle
    prevAngles[0] = 300;
    prevAngles[1] = 300;
    
    // TODO wrap the whole method in a loop while the robot is rotating
    // usMotor.rotate(360, true);
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.rotate(-90, false);
    usMotor.forward();
    double startTacho = usMotor.getTachoCount();

    
    // arbitrary prev angle to prevent from double reading
    double prevAngle = -10;

    while (startTacho < 90) {
      double angle = (odometer.getXyt()[2] - usMotor.getTachoCount() + 360) % 360;

      int objDist = readUsDistance();
      // Throw out objects over 2 tile distances away
      //System.out.println(objDist);
      if (detectObjInPath() && angle != prevAngle) {
        // Stop rotation and latch onto object, determine width
        // System.out.println("Angle " + angle + " PrevAngle " + prevAngle);
        // Detect objects in a 1 tile radius
        usMotor.stop();
        isBlock = detectBlock(objDist);
        
        if (isBlock) {
          angleMap.put(angle, objDist);
        }
        usMotor.setSpeed(ROTATE_SPEED);
        usMotor.forward();
      }

      prevAngle = angle;

      // Get the current tachocount
      startTacho = usMotor.getTachoCount();
    }
    usMotor.stop();
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
  public static boolean detectBlock(double objDist) {
    /*
     * if not in a certain threshold then the object is not a block
     */
    double THRESHOLD = 15;
    
    double maxTreshold  = objDist;
    
    
    // TODO ROTATE TO FIND EDGE
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.backward();
    while (objDist <= DETECTION_THRESHOLD) {
      objDist = readUsDistance();
    }
    
    maxTreshold =objDist;
    usMotor.stop();
    double angle1 = -usMotor.getTachoCount();

    double tempTacho = usMotor.getTachoCount();
    // TODO ROTATE OPPOSITE DIRECTION TO FIND OTHER EDGE
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.forward();
    while (((objDist <= maxTreshold && usMotor.getTachoCount() < 90)
        || usMotor.getTachoCount() < tempTacho + 10)) {
      objDist = readUsDistance();
    }
    usMotor.stop();
    double angle2 = -usMotor.getTachoCount();


//    // Throw out last value
//    if (prevAngles[1] == angle1 && isBlock) {
//      angle1 = prevAngles[0];
//      ArrayList<Double> keyList = new ArrayList<>(angleMap.keySet());
//      angleMap.remove(keyList.get(keyList.size() - 1));
//    }
    
    if(prevAngles[1] == angle2) {
      return false;
    }
    
    if(angle1 >= prevAngles[1]) {
      angle1 = prevAngles[0];
      if(isBlock) {
        ArrayList<Double> keyList = new ArrayList<>(angleMap.keySet());
        angleMap.remove(keyList.get(keyList.size() - 1));
      }
    }
    
    System.out.println("theta 1 " + angle1 + " theta 2 " + angle2 + "previous angle 2  " + prevAngles[1] + "  " + isBlock);
    prevAngles[0] = angle1;
    prevAngles[1] = angle2;
    if (Math.abs(angle1 - angle2) > THRESHOLD) {
      return false;
    }

   

    return true;
  }


  public static boolean detectObjInPath() {
    int objDist = readUsDistance();
    // Object out of detection range
    if (objDist > DETECTION_THRESHOLD) {
      return false;
    }
    // System.out.println(objDist);
    double X = odometer.getXyt()[0];
    double Y = odometer.getXyt()[1];
    double angle = Math.toRadians((odometer.getXyt()[2] - usMotor.getTachoCount()));

    // Throw out false positives (eg: walls, bins, tunnel)
    // TODO determine the bounds of the map

    // Calculate final point coordinates based on distance
    double XF = X + Math.sin(angle) * objDist / 100.0;
    double YF = Y + Math.cos(angle) * objDist / 100.0;
    angle = Math.toDegrees(angle);
    // System.out.println(angle);
    // System.out.println(objDist);
    // System.out.println("X " + X + " Y " + Y);
    // System.out.println("XF " + XF + " YF " + YF);

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
    if (XF >= 15 * (TILE_SIZE) || XF <= 0 || (YF >= 9 * TILE_SIZE) || YF <= 0) {
      return false;
    }

    // Check if any points are bin/tunnel coordinates
    Point p = new Point(Math.round(XF), Math.round(YF));

    return true;
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
