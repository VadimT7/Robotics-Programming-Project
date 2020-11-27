package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.RampEdge;

public class Navigation {


  /** Do not instantiate this class. */
  private Navigation() {}

  /** Travels to the given destination. */
  public static void travelTo(Point destination) {
    var xyt = odometer.getXyt();
    var currentLocation = new Point(xyt[0] / TILE_SIZE, xyt[1] / TILE_SIZE);
    var currentTheta = xyt[2];
    var destinationTheta = getDestinationAngle(currentLocation, destination);
    System.out.println(destinationTheta);
    Driver.turnBy(minimalAngle(currentTheta, destinationTheta));
    Driver.moveStraightFor(distanceBetween(currentLocation, destination));
  }

  /** travels across the tunnel based on the given coordinates. */
  public static void travelAcrossTunnel() {

    Point ll;
    Point up;
    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

    // Pick depending on starting color
    if (STARTING_COLOR.equals("red")) {
      ll = tnr.ll;
      up = tnr.ur;
    } else {
      ll = tng.ll;
      up = tng.ur;
    }

    Point p1;
    Point p2;
    Point p3;

    double angle = odometer.getXyt()[2];

    // Travel to the tunnel based on the location of the island
    // Checks to see if the tunnel is along the x axis
    if (Math.abs(ll.x - up.x) > 1) {
      if (up.x == island.ll.x) {
        p1 = new Point(ll.x - 1, ll.y + 1);
        p2 = new Point(ll.x - 1, ll.y + 0.45);
        p3 = new Point(up.x + 1, ll.y + 0.45);
      } else {
        p1 = new Point(up.x - 1, ll.y - 1);
        p2 = new Point(up.x - 1, up.y - 0.45);
        p3 = new Point(ll.x + 1, up.y - 0.45);
      }
    } else {
      if (up.y == island.ll.y) {
        p1 = new Point(ll.x + 1, ll.y - 1);
        p2 = new Point(ll.x + 0.5, ll.y - 1);
        p3 = new Point(ll.x + 0.5, up.y + 0.5);
      } else {
        p1 = new Point(ll.x + 1, up.y + 1);
        p2 = new Point(ll.x + 0.5, up.y + 1);
        p3 = new Point(ll.x + 0.5, ll.y - 1);
      }
    }

    // Travel to lower left and localize, avoid if object in path
    turnTo(getDestinationAngle(current, p1));
    ObjectDetection.OutobjectAvoider(p1);
    turnTo(0);
    LightLocalizer.localize(p1.x * TILE_SIZE, p1.y * TILE_SIZE, 0);
    // find magnitude of length across grid that the robot will travel from initial point to dest.
    travelTo(p2);
    // Travel across tunnel in a straight line with line detection
    angle = getDestinationAngle(p2, p3);
    turnTo(angle);
    LightLocalizer.lineDetect();
    travelTo(p3);
  }



  public static void travelToSearchZone() {
    Point ll;
    Point ur;
    if (STARTING_COLOR.equals("red")) {
      ll = szr.ll;
      ur = szr.ur;
    } else {
      ll = szg.ll;
      ur = szg.ur;
    }
    Point p1 = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    double curX = odometer.getXyt()[0] / TILE_SIZE;
    double curY = odometer.getXyt()[1] / TILE_SIZE;

    if (!((curX >= ll.x && curX <= ur.x) && (curY <= ur.y && curY >= ll.y))) {
      double angle = Navigation.getDestinationAngle(p1, new Point(ll.x + 2, szg.ll.y + 1));
      Navigation.turnTo(angle);
      ObjectDetection.OutobjectAvoider(new Point(ll.x + 2, ll.y + 1));
    }
    LightLocalizer.robotBeep(3);
  }


  public static void travelToRampAndBack() {
    Point ramp;

    if (STARTING_COLOR.equals("red")) {
      ramp = new Point(rr.left.x + 0.5, rr.left.y - 0.5);
    } else {
      ramp = new Point(gr.left.x + 0.5, gr.left.y - 0.5);
    }

    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    double angle = Navigation.getDestinationAngle(current, ramp);
    Navigation.turnTo(angle);
    ObjectDetection.OutobjectAvoider(ramp);
    // localize to a corner
    LightLocalizer.localize();
    // LightLocalizer.localize((rr.right.x), (rr.right.y - 1)*TILE_SIZE, odometer.getXyt()[2]);

    // move backwards half a tile
    Driver.setSpeed(FORWARD_SPEED);
    Driver.moveStraightFor(-0.5 * TILE_SIZE); // move back half a tile

    // turn to face the ramp
    Driver.turnBy(-90);

    // move forwards till the ramp is detected by the two light sensors in the back
    LightLocalizer.lineDetect();

    // push the box up the ramp and descend back to the start of the ramp
    pushObjectOnRampAndReturn();

  }

  public static void pushToRamp() {

    RampEdge ramp;

    double turningAngle = 0;
    double endX;
    double endY;

    // Length of the bin/ramp along the y axis
    double maxY;

    if (STARTING_COLOR.equals("red")) {
      ramp = rr;
    } else {
      ramp = gr;
    }

    // Change pushing based on where the ramp is located
    if (ramp.left.x > ramp.right.x) {
      endX = ramp.right.x + 0.5;
      endY = ramp.left.y + 1;
      maxY = ramp.left.y - 2;
    } else if (ramp.left.x < ramp.right.x) {
      endX = ramp.right.x - 0.5;
      endY = ramp.left.y - 1;
      maxY = ramp.left.y + 2;
    } else if (ramp.left.y < ramp.right.y) {
      endX = ramp.right.x + 1;
      endY = ramp.left.y + 0.5;
      maxY = ramp.left.y;
    } else {
      endX = ramp.right.x - 1;
      endY = ramp.left.y - 0.5;
      maxY = ramp.left.y;
    }

    // Initial position (where the block is)
    double x = odometer.getXyt()[0];
    double y = odometer.getXyt()[1];
    
    // Decide to travel x or y first
    // Robot falls in between the bins position
    // travel along y first
    // double rampMin = Math.min(ramp.left.y, ramp.right.y);
    // if (maxY <= cur.y && rampMin >= cur.y) {
    // if (cur.y < endY) {
    // travelTo(new Point(cur.x, cur.y + 0.5));
    // turningAngle = 180;
    // } else {
    // travelTo(new Point(cur.x, cur.y - 0.5));
    // turningAngle = 0;
    // }
    // } else {
    // if (cur.x < endX) {
    // travelTo(new Point(cur.x - 0.5, cur.y - 0.5));
    // turningAngle = 90;
    // } else {
    // travelTo(new Point(cur.x + 0.5, cur.y - 0.5));
    // turningAngle = 270;
    // }
    // }
    // turnTo(turningAngle);
    // Driver.moveStraightFor(0.5);

    // Turn to and check for object
    // Keep calling travel method while the robot has not reached its destination
    // while (cur.x != endX && cur.y != endY) {
    // // Update the position
    // pushWithObjDetect(cur, new Point(endX, endY));
    // x = Math.round(odometer.getXyt()[0]);
    // y = Math.round(odometer.getXyt()[1]);
    // cur = new Point(x / TILE_SIZE, y / TILE_SIZE);
    // }
  }

  /**
   * Method that allows the robot to push the box to the top of the ramp and then descend to its starting position
   * (bottom of the ramp).
   */
  public static void pushObjectOnRampAndReturn() {
    Point rampStart;
    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);

    if (STARTING_COLOR.equals("red")) {
      rampStart = new Point(rr.left.x + 0.5, rr.left.y - 0.5);
    } else {
      rampStart = new Point(gr.left.x + 0.5, gr.left.y - 0.5);
    }

    // push the object up the ramp until the edge of the ramp is detected
    LightLocalizer.rampEndDetect();

    // return to the bottom of the ramp
    // (optionally) turn to face the bottom of the ramp
    double angle = Navigation.getDestinationAngle(current, rampStart);
    Navigation.turnTo(angle);

    // move down the ramp - (alternative implementation if the robot slips while turning with code right above (turnTo):
    // move backwards without turning)
    double distanceToBottomOfRamp = Navigation.distanceBetween(current, rampStart);
    Driver.moveStraightFor(distanceToBottomOfRamp);

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
