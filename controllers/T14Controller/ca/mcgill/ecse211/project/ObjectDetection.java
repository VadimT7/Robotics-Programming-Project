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
    usMotor.rotate(-90, false);
    usMotor.forward();
    double startTacho = usMotor.getTachoCount();
    
    //arbitrary prev angle to prevent from double reading
    double prevAngle = -10;
    
    while (startTacho <= 90) {
      double angle = (odometer.getXyt()[2] - usMotor.getTachoCount() + 360) % 360;

      int objDist = readUsDistance();
      // Throw out objects over 2 tile distances away
      if (detectObjInPath(objDist) && angle != prevAngle) {
        // Stop rotation and latch onto object, determine width
        //System.out.println("Angle " + angle + " PrevAngle " + prevAngle);
        usMotor.stop();
        boolean isBlock = detectBlock();
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

    System.out.println(usMotor.getTachoCount());
    usMotor.stop();
    usMotor.resetTachoCount();
    return angleMap;
  }

  /**
   * 
   * @return is the object a block
   */
  public static boolean detectBlock() {
    
    //System.out.println("Tacho start" + startTacho);
    /*
     * if not in a certain threshold then the object is not a block
     */
    int objDist = readUsDistance();

    double THRESHOLD = 30;
    
    
    // TODO ROTATE OPPOSITE DIRECTION TO FIND OTHER EDGE
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.forward();
    while ((objDist <= 2 * TILE_SIZE * 100 && usMotor.getTachoCount() <= 90)
        ) {
      objDist = readUsDistance();
    }

    int startTacho = usMotor.getTachoCount();
    usMotor.stop();
    double angle2 = (-usMotor.getTachoCount() + 360) % 360;
    
    // TODO ROTATE TO FIND EDGE
    double tempTacho = usMotor.getTachoCount();
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.backward();
    while (objDist <= 2 * TILE_SIZE * 100||  usMotor.getTachoCount() > tempTacho - 10) {
      objDist = readUsDistance();
    }
    usMotor.stop();
    double angle1 = (-usMotor.getTachoCount() + 360) % 360;
    

    int endTacho = usMotor.getTachoCount();

    
    // Rotate back to initial position
    int tachoDiff = 0;
    if(startTacho < 0) {
      startTacho = (startTacho > 0)? startTacho:-startTacho;
      tachoDiff = startTacho;
    }else {
      tachoDiff = startTacho - endTacho;
    }
    
    
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.rotate(tachoDiff, false);
    if (Math.abs(angle1 - angle2) > THRESHOLD) {
      return false;
    }

    return true;
  }


  public static boolean detectObjInPath(int objDist) {
    // Object out of detection range
    if (objDist > DETECTION_THRESHOLD) {
      return false;
    }
    // System.out.println(objDist);
    double X = odometer.getXyt()[0];
    double Y = odometer.getXyt()[1];
    double angle = (odometer.getXyt()[2] - usMotor.getTachoCount() + 360) % 360;


    // Throw out false positives (eg: walls, bins, tunnel)
    // TODO determine the bounds of the map

    // Calculate final point coordinates based on distance
    double XF = X + Math.sin(angle) * objDist / 100.0;
    double YF = Y + Math.cos(angle) * objDist / 100.0;
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
