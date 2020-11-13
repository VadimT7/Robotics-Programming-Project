package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.DETECTION_THRESHOLD;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.tng;
import static ca.mcgill.ecse211.project.Resources.tnr;
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



public class ObjectDetection {

  private static LinkedHashMap<Double, Integer> angleMap;
  private static boolean isBlock;
  private static double[] prevAngles = new double[2];

  public static LinkedHashMap<Double, Integer> findObjects() {
    angleMap = new LinkedHashMap<>();
    
    //Set random prevangle
    prevAngles[0] = 300;
    prevAngles[1] = 300;
    
    //Rotate to 90 degrees to begin the 180 degree sweep
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.rotate(-90, false);
    usMotor.forward();
        
    //Save the tacho count of the ultrasonic sensor motor
    double startTacho = usMotor.getTachoCount();

    
    // arbitrary prev angle to prevent from reading the same angle twice
    double prevAngle = -10;

    while (startTacho < 90) {
      double angle = (odometer.getXyt()[2] - usMotor.getTachoCount() + 360) % 360;

      int objDist = readUsDistance();
      // Throw out objects over 2 tile distances away/ at the same angle
      if (detectObjInPath() && angle != prevAngle) {
        // Stop rotation and latch onto object, determine width
        usMotor.stop();
        isBlock = detectBlock(objDist);
        
        //If the object detected is a block then add it to the map
        if (isBlock) {
          angleMap.put(angle, objDist);
        }
        //Continue the rotation
        usMotor.setSpeed(ROTATE_SPEED);
        usMotor.forward();
      }
      
      //Save the previous angle
      prevAngle = angle;

      // Get the current tachocount
      startTacho = usMotor.getTachoCount();
    }
    
    //Stop the rotation
    usMotor.stop();
    
    //Have the motor rotate back to 0 degrees (where the robot is facing)
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
    
    
    //Rotate to find the edge of the object of the object
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.backward();
    while (objDist <= DETECTION_THRESHOLD) {
      objDist = readUsDistance();
    }
    maxTreshold = objDist;
    usMotor.stop();
    
    //Save the angle
    double angle1 = -usMotor.getTachoCount();

    double tempTacho = usMotor.getTachoCount();
    // Rotate opposite direction to find the other edge
    usMotor.setSpeed(ROTATE_SPEED);
    usMotor.forward();
    while (((objDist <= maxTreshold && usMotor.getTachoCount() < 90)
        || usMotor.getTachoCount() < tempTacho + 10)) {
      objDist = readUsDistance();
    }
    usMotor.stop();
    
    //Save the second angle
    double angle2 = -usMotor.getTachoCount();

    // Throw out value if this second angle was the same as the previous
    if(prevAngles[1] == angle2) {
      return false;
    }
    
    //Remove previous value if there is overlap between the angles
    if(angle1 >= prevAngles[1]) {
      angle1 = prevAngles[0];
      if(isBlock) {
        ArrayList<Double> keyList = new ArrayList<>(angleMap.keySet());
        angleMap.remove(keyList.get(keyList.size() - 1));
      }
    }
    
    System.out.println("theta 1 " + angle1 + " theta 2 " + angle2 + "previous angle 2  " + prevAngles[1] + "  " + isBlock);
    
    //Save the angles
    prevAngles[0] = angle1;
    prevAngles[1] = angle2;
    
    //Verify that the width is under a certain threshold
    if (Math.abs(angle1 - angle2) > THRESHOLD) {
      return false;
    }

    return true;
  }

  /**
   * Method checks if an object has been detected within a 2 tile radius.
   * This method throws out values if they correspond to ramps, tunnels, bins
   * 
   * @return if an object has been detected
   */
  public static boolean detectObjInPath() {
    int objDist = readUsDistance();
    // Object out of detection range
    if (objDist > DETECTION_THRESHOLD) {
      return false;
    }
    
    //Retrieve the current position and angle
    double X = odometer.getXyt()[0];
    double Y = odometer.getXyt()[1];
    double angle = Math.toRadians((odometer.getXyt()[2] - usMotor.getTachoCount()));

    // Calculate final point coordinates based on distance
    double XF = X + Math.sin(angle) * objDist / 100.0;
    double YF = Y + Math.cos(angle) * objDist / 100.0;
    angle = Math.toDegrees(angle);

    // Detected a wall
    if (XF >= 15 * (TILE_SIZE) || XF <= 0 || (YF >= 9 * TILE_SIZE) || YF <= 0) {
      return false;
    }

    // Check if any points are bin/tunnel coordinates, these are false positives
    // Check if it matches the red tunnel
    if(XF >= tnr.ll.x && XF <=tnr.ur.x && YF >= tnr.ll.y && YF <= tnr.ur.y) {
      return false;
    }

    //Check if it matches the green tunnel
    if(XF >= tng.ll.x && XF <=tng.ur.x && YF >= tng.ll.y && YF <= tng.ur.y) {
      return false;
    }
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
