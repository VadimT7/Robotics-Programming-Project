package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;

import ca.mcgill.ecse211.playingfield.Point;

public class Navigation {

  /** Do not instantiate this class. */
  private Navigation() {}

  /** Travels to the given destination. */
  public static void travelTo(Point destination) {
    var xyt = odometer.getXyt();
    var currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    var currentTheta = xyt[2];
    var destinationTheta = getDestinationAngle(currentLocation, destination);
    Driver.turnBy(minimalAngle(currentTheta, destinationTheta));
    Driver.moveStraightFor(distanceBetween(currentLocation, destination));
  }

  /** travels across the tunnel based on the given coordinates. */
  public static void travelAcrossTunnel() {

    Point ll;
    Point up;

    // Pick depending on starting color
    if (STARTING_COLOR.equals("red")) {
      ll = tnr.ll;
      up = tnr.ur;
    } else {
      ll = tng.ll;
      up = tng.ur;
    }
    //TODO think about 2 other cases for the tunnel
    
    
    // If there is a difference of more than 1 between the x of the LL AND UP coordinates
    Point p2;
    Point p3;

    if (Math.abs(ll.x - up.x) > 1) {
      p2 = new Point(ll.x - 1, ll.y + 0.4);
      p3 = new Point(up.x + 1, ll.y + 0.4);
    } else {
      p2 = new Point(ll.x + 0.4, ll.y - 1);
      p3 = new Point(ll.x + 0.4, up.y + 1);
    }

    // find magnitude of length across grid that the robot will travel from initial point to dest.
    travelWithObjDetect(p2);

    // Travel across tunnel in a straight line with line detection
    double disToTravel = distanceBetween(p2, p3);
    double angle = getDestinationAngle(p2, p3);
    turnTo(angle);
    //LightLocalizer.lineDetect();
    travelTo(p3);
  }

  public static void travelWithObjDetect(Point destination) {
    Point p1 = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

    double angle = getDestinationAngle(p1, destination);
    turnTo(angle);
    int objDist = UltrasonicLocalizer.readUsDistance();
    // Consider an object if it is within 2 tiles
    if (ObjectDetection.detectObjInPath(objDist)) {
      // Need to know the initial position of the robot
      travelTo(new Point(destination.x, p1.y));
      travelTo(new Point(destination.x, destination.y));
    }else {
      travelTo(destination);
    }

  }

  public static void travelToSearchZone(){
    Point ll;
    Point ur;

    // Pick depending on starting color
    if (STARTING_COLOR.equals("red")) {
      ll = szr.ll;
      ur = szr.ur;
    } else {
      ll = szg.ll;
      ur = szg.ur;
    }
    
    double curX = odometer.getXyt()[0];
    double curY = odometer.getXyt()[1];
    
    if(!((curX >= ll.x && curX <= ur.x) && (curY <= ur.y && curY >= ll.y))) {
      
    }
    
    
  }
  /** Returns the angle that the robot should point towards to face the destination in degrees. */
  public static double getDestinationAngle(Point current, Point destination) {
    return (toDegrees(atan2(destination.x - current.x, destination.y - current.y)) + 360) % 360;
  }

  /** Returns the signed minimal angle from the initial angle to the destination angle. */
  public static double minimalAngle(double initialAngle, double destAngle) {
    var dtheta = destAngle - initialAngle;
    if (dtheta < -180) {
      dtheta += 360;
    } else if (dtheta > 180) {
      dtheta -= 360;
    }
    return dtheta;
  }

  /** Returns the distance between the two points in tile lengths. */
  public static double distanceBetween(Point p1, Point p2) {
    var dx = p2.x - p1.x;
    var dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
  }

  /**
   * Turns the robot with a minimal angle towards the given input angle in degrees, no matter what its current
   * orientation is. This method is different from {@code turnBy()}.
   * 
   * @param angle angle to turn the robot to
   */
  public static void turnTo(double angle) {
    Driver.turnBy(minimalAngle(odometer.getXyt()[2], angle));
  }



}
