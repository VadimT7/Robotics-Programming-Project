package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static java.lang.Math.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.readUsDistance;
import java.util.Map.Entry;
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

    turnTo(getDestinationAngle(current, p1));
    ObjectDetection.objectAvoider(p1);
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

  /**
   * Have the robot travel in straight lines with object detection
   * @param destination the point the robot needs to push the block to
   * @param ramp the ramp coordinates
   * @param rampOrientation direction of the ramp
   */
  public static void pushWithObjDetect(Point destination, RampEdge ramp, int rampOrientation) {
    // Starting point is where the block is placed
    Point start = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    
    double maxY;
    double minY;
    // Change pushing based on where the ramp is located
    if (ramp.left.x > ramp.right.x) {
      minY = ramp.left.y;
      maxY = ramp.left.y - 2;
    } else if (ramp.left.x < ramp.right.x) {
      minY = ramp.left.y;
      maxY = ramp.left.y + 2;
    } else if (ramp.left.y < ramp.right.y) {
      minY = ramp.left.y;
      maxY = ramp.right.y;
    } else {
      minY = ramp.right.y;
      maxY = ramp.left.y;
    }

    // The robot is located besides the bin move to
    if (start.y >= minY && start.y <= maxY) {
      // Move until we pass the bin
      travelTo(new Point(start.x, destination.y));
      // Move to push the block sideways
      double turningAngle;

      start = odometer.getPoint();
      Driver.moveStraightFor(-0.5);
      if (start.x < destination.x) {
        if (odometer.getPoint().y > destination.y) {
          ObjectDetection.objectAvoider(new Point(start.x - 0.25, start.y - 0.25));
        } else {
          ObjectDetection.objectAvoider(new Point(start.x - 0.25, start.y + 0.25));
        }
        turningAngle = 90;
      } else {
        if (odometer.getPoint().y > destination.y) {
          ObjectDetection.objectAvoider(new Point(start.x + 0.25, start.y - 0.25));
        } else {
          ObjectDetection.objectAvoider(new Point(start.x + 0.25, start.y + 0.25));
        }
        turningAngle = 270;
      }
      turnTo(turningAngle);
      LightLocalizer.localize();
      start = odometer.getPoint();
    }


    double angle = odometer.getXyt()[2];

    double objDist = readUsDistance();
    if (objDist <= DETECTION_THRESHOLD) {
      // Have the robot move back to check its surroundings
      Driver.moveStraightFor(-0.6);
      usMotor.rotate(45, false);
      double turningAngle = 0;
      // Check for object
      if (!(readUsDistance() <= DETECTION_THRESHOLD)) {
        
        //Move to the side of the block and push it away from the obstacle
        if (angle >= -10 && angle <= 10) {
          travelTo(new Point(start.x + 0.75, start.y + 0.25));
          turningAngle = 270;
        } else if (angle >= 90 - 10 && angle <= 90 + 10) {
          travelTo(new Point(start.x + 0.75, start.y - 0.25));
          turningAngle = 180;
        } else if (angle >= 180 - 10 && angle <= 180 + 10) {
          travelTo(new Point(start.x - 0.5, start.y - 0.5));
          turningAngle = 90;
        } else {
          travelTo(new Point(start.x - 0.5, start.y + 0.5));
        }

      } else {
        //Move to opposite side since there was an object detected here
        if (angle >= -10 && angle <= 10) {
          travelTo(new Point(start.x - 0.75, start.y + 0.25));
          turningAngle = 270;
        } else if (angle >= 90 - 10 && angle <= 90 + 10) {
          travelTo(new Point(start.x + 0.75, start.y + 0.25));
          turningAngle = 180;
        } else if (angle >= 180 - 10 && angle <= 180 + 10) {
          travelTo(new Point(start.x + 0.5, start.y - 0.5));
          turningAngle = 90;
        } else {
          travelTo(new Point(start.x - 0.5, start.y - 0.5));
        }
      }
      usMotor.rotate(-45, false);
      // Turn back towards the block
      turnTo(turningAngle);
      Driver.moveStraightFor(2);
    } 
    
    //No object was detected
    else {
      double xOff = destination.x - 0.5;
      angle = odometer.getXyt()[2];
      if (angle <= 360 && angle >= 180) {
        xOff = destination.x + 0.5;
      }
      
      //Push the block to the x of the ramp
      if (verifyThreshold(start.x, xOff)) {
        travelTo(new Point(xOff, start.y));
      } 
      
      //Push the block along the y direction towards the bin
      else {
        Point p = odometer.getPoint();
        Driver.moveStraightFor(-0.5);
        double yOff = p.y - 0.5;
        if (rampOrientation > 2) {
          yOff = p.y + 0.5;
        }
        travelTo(new Point(destination.x, yOff));
        turnTo(getDestinationAngle(odometer.getPoint(), destination));
        Driver.moveStraightFor(0.4);
        
        
        //Detected object need to push the block away
        if (readUsDistance() < DETECTION_THRESHOLD) {
          pushWithObjDetect(destination, ramp, rampOrientation);
          p = odometer.getPoint();
          Driver.moveStraightFor(-0.5);
          yOff = p.y - 0.75;
          if (rampOrientation > 2) {
            yOff = p.y + 0.75;
          }
          xOff = p.x + 0.5;
          angle = odometer.getXyt()[2];
          if (angle <= 360 && angle >= 180) {
            xOff = p.x - 0.5;
          }
          travelTo(new Point(xOff, yOff));
          turnTo(getDestinationAngle(odometer.getPoint(), new Point(odometer.getPoint().x, destination.y)));
          Driver.moveStraightFor(3);
          
          Point cur = odometer.getPoint();
          Driver.moveStraightFor(-0.5);
          double turningAngle;
          
          //Move to push the block along the x axis
          if (cur.x < destination.x) {
            ObjectDetection.objectAvoider(new Point(cur.x - 0.5, cur.y));
            turningAngle = 90;
          } else {
            ObjectDetection.objectAvoider(new Point(cur.x + 0.5, cur.y));
            turningAngle = 270;
          }
          turnTo(turningAngle);
          
        } else {
          travelTo(destination);
        }

      }
    }
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
      ObjectDetection.objectAvoider(new Point(ll.x + 2, ll.y + 1));
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
    ObjectDetection.objectAvoider(ramp);
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


  public static void moveToBlock(Entry<Double, Integer> entry) {
    // The current angle the robot is facing
    double curAngle = odometer.getXyt()[2];

    // Extra turning the robot might need to accommodate the sensor position
    double angleOffset = 10;

    // if (minimalAngle(curAngle, entry.getValue()) + curAngle > curAngle) {
    // angleOffset = 10;
    // }

    turnTo(entry.getKey() + angleOffset);


    // Move straight until the torque changes
    // Driver.moveStraightFor(entry.getValue() / (TILE_SIZE * 100));
    Point start = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    Point current = start;
    while (distanceBetween(start, current) < entry.getValue() / (100 * TILE_SIZE)) {
      Driver.forward();
      // If the torque is over a certain threshold, the robot is pushing a block
      if ((rightMotor.getTorque() + leftMotor.getTorque()) / 2 * 100 > 20 && distanceBetween(start, current) > 0.1) {
        break;
      }

      current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    }
    Driver.stopMotors();

  }


  /**
   * Pushes a box from a starting point to an end point
   *
   */
  public static void pushTo() {

    RampEdge ramp;

    double turningAngle;

    double endX;
    double endY;

    // Length of the bin/ramp along the y axis
    double maxY;

    int rampCase;

    if (STARTING_COLOR.equals("red")) {
      ramp = rr;
    } else {
      ramp = gr;
    }

    // Change pushing based on where the ramp is located
    if (ramp.left.x > ramp.right.x) {
      endX = ramp.left.x - 0.5;
      endY = ramp.left.y + 0.5;
      maxY = ramp.left.y - 2;
      rampCase = 1;
    } else if (ramp.left.x < ramp.right.x) {
      endX = ramp.right.x - 0.5;
      endY = ramp.left.y - 0.5;
      maxY = ramp.left.y + 2;
      rampCase = 2;
    } else if (ramp.left.y < ramp.right.y) {
      endX = ramp.right.x + 0.5;
      endY = ramp.left.y + 0.5;
      maxY = ramp.right.y;
      rampCase = 3;
    } else {
      endX = ramp.right.x - 0.5;
      endY = ramp.left.y - 0.5;
      maxY = ramp.left.y;
      rampCase = 4;
    }

    // Initial position (where the block is)
    double x = odometer.getXyt()[0];
    double y = odometer.getXyt()[1];
    Point cur = new Point(x / TILE_SIZE, y / TILE_SIZE);


    // Should be starting at the block
    // Move back half a tile
    Driver.moveStraightFor(-0.5);

    // Decide to travel x or y first
    // Robot falls in between the bins position
    // travel along y first

    // Get the current position of the robot
    double rampMin = Math.min(ramp.left.y, ramp.right.y);
    if (maxY >= cur.y && rampMin <= cur.y) {
      if (cur.y >= endY) {
        ObjectDetection.objectAvoider(new Point(cur.x, cur.y + 1.25));
        turningAngle = 180;
      } else {
        ObjectDetection.objectAvoider(new Point(cur.x, cur.y - 0.5));
        turningAngle = 0;
      }
    } else {
      if (cur.x < endX) {
        ObjectDetection.objectAvoider(new Point(cur.x - 0.5, cur.y));
        turningAngle = 90;
      } else {
        ObjectDetection.objectAvoider(new Point(cur.x + 0.5, cur.y));
        turningAngle = 270;
      }
    }
    turnTo(turningAngle);
    Driver.moveStraightFor(0.5);

    // Turn to and check for object
    // Keep calling travel method while the robot has not reached its destination
    while (verifyThreshold(cur.x, endX) || verifyThreshold(cur.y, endY)) {
      pushWithObjDetect(new Point(endX, endY), ramp, rampCase);
      cur = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    }
    
    pushObjectOnRampAndReturn();
    Driver.stopMotors();
  }

  private static boolean verifyThreshold(double cur, double end) {
    return !(cur <= end + 0.5) || !(cur >= end - 0.5);
  }

  /**
   * Method that allows the robot to push the box to the top of the ramp and then descend to its starting position
   * (bottom of the ramp).
   */
  public static void pushObjectOnRampAndReturn() {
    Point rampStart;

    if (STARTING_COLOR.equals("red")) {
      rampStart = new Point(rr.left.x + 0.5, rr.left.y - 0.5);
    } else {
      rampStart = new Point(gr.left.x + 0.5, gr.left.y - 0.5);
    }
    
//    LightLocalizer.lineDetect();

    // push the object up the ramp until the edge of the ramp is detected
    LightLocalizer.rampEndDetect();
    Point current = new Point(odometer.getXyt()[0] / TILE_SIZE, odometer.getXyt()[1] / TILE_SIZE);
    Driver.moveStraightFor(1);

    // return to the bottom of the ramp
    // move down the ramp
    double distanceToBottomOfRamp = Navigation.distanceBetween(current, rampStart);
    Driver.moveStraightFor(-distanceToBottomOfRamp);

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
