package ca.mcgill.ecse211.project;

import java.math.BigDecimal;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.RampEdge;
import ca.mcgill.ecse211.playingfield.Region;
import ca.mcgill.ecse211.wificlient.WifiConnection;
import simlejos.hardware.motor.Motor;
import simlejos.hardware.port.SensorPort;
import simlejos.hardware.sensor.EV3ColorSensor;
import simlejos.hardware.sensor.EV3UltrasonicSensor;
import simlejos.robotics.RegulatedMotor;

/* (non-Javadoc comment)
 * TODO Integrate this carefully with your existing Resources class (See below for where to add
 * your code from your current Resources file). The order in which things are declared matters!
 */

/**
 * Class for static resources (things that stay the same throughout the entire program execution),
 * like constants and hardware.
 * <br><br>
 * Use these resources in other files by adding this line at the top (see examples):<br><br>
 * 
 * {@code import static ca.mcgill.ecse211.project.Resources.*;}
 */
public class Resources {
  
  // Wi-Fi client parameters
  /** The default server IP used by the profs and TA's. */
  public static final String DEFAULT_SERVER_IP = "127.0.0.1";

  /**
   * The IP address of the server that sends data to the robot. For the beta demo and competition,
   * replace this line with
   * 
   * <p>{@code public static final String SERVER_IP = DEFAULT_SERVER_IP;}
   */
  public static final String SERVER_IP = "127.0.0.1"; // = DEFAULT_SERVER_IP;

  /** Your team number. */
  public static final int TEAM_NUMBER = 0; // TODO

  /** Enables printing of debug info from the WiFi class. */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /** Enable this to attempt to receive Wi-Fi parameters at the start of the program. */
  public static final boolean RECEIVE_WIFI_PARAMS = true;


  // Simulation-related constants
  
  /** The time between physics steps in milliseconds. */
  public static final int PHYSICS_STEP_PERIOD = 500; // ms
  
  /** The relative path of the input vector file. */
  public static final Path VECTORS_FILE = Paths.get("vectors.txt");


  //----------------------------- DECLARE YOUR CURRENT RESOURCES HERE -----------------------------
  //----------------------------- eg, constants, motors, sensors, etc -----------------------------


  // Robot constants
  
  /** The maximum distance detected by the ultrasonic sensor, in cm. */
  public static final int MAX_SENSOR_DIST = 255;
  
  /** The limit of invalid samples that we read from the US sensor before assuming no obstacle. */
  public static final int INVALID_SAMPLE_LIMIT = 0;
  
  /** The wheel radius in meters. */
  public static final double WHEEL_RAD = 0;
  
  /** The robot width in meters. */
  public static final double BASE_WIDTH = 0;
  
  /** The distance between the color sensors and the wheels in meters. */
  public static final double COLOR_SENSOR_TO_WHEEL_DIST = 0;
  
  /** The speed at which the robot moves forward in degrees per second. */
  public static final int FORWARD_SPEED = 0;
  
  /** The speed at which the robot rotates in degrees per second. */
  public static final int ROTATE_SPEED = 0;
  
  /** The motor acceleration in degrees per second squared. */
  public static final int ACCELERATION = 0;
  
  /** Timeout period in milliseconds. */
  public static final int TIMEOUT_PERIOD = 3000;
  
  /** The tile size in meters. Note that 0.3048 m = 1 ft. */
  public static final double TILE_SIZE = 0.3048;


  // Hardware resources

  /** The left motor. */
  public static final RegulatedMotor leftMotor = Motor.A;
  
  /** The right motor. */
  public static final RegulatedMotor rightMotor = Motor.D;
  
  /** The ultrasonic sensor. */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /** The left color sensor. */
  public static final EV3ColorSensor leftColorSensor = new EV3ColorSensor(SensorPort.S2);
  
  /** The right color sensor. */
  public static final EV3ColorSensor rightColorSensor = new EV3ColorSensor(SensorPort.S3);


  // Software singletons
  
  /** The odometer. */
  public static Odometer odometer = Odometer.getOdometer();


  // Wi-Fi parameters

  /** Container for the Wi-Fi parameters. */
  public static Map<String, Object> wifiParameters;
  
  // This static initializer MUST be declared before any Wi-Fi parameters.
  static {
    receiveWifiParameters();
  }
  
  /** Red team number. */
  public static int redTeam = getWP("RedTeam");

  /** Red team's starting corner. */
  public static int redCorner = getWP("RedCorner");

  /** Green team number. */
  public static int greenTeam = getWP("GreenTeam");

  /** Green team's starting corner. */
  public static int greenCorner = getWP("GreenCorner");

  /** The edge when facing the Red ramp. */
  public static RampEdge rr = makeRampEdge("RR");

  /** The edge when facing the Green ramp. */
  public static RampEdge gr = makeRampEdge("GR");

  /** The Red Zone. */
  public static Region red = makeRegion("Red");

  /** The Green Zone. */
  public static Region green = makeRegion("Green");

  /** The Island. */
  public static Region island = makeRegion("Island");

  /** The red tunnel footprint. */
  public static Region tnr = makeRegion("TNR");

  /** The green tunnel footprint. */
  public static Region tng = makeRegion("TNG");

  /** The red search zone. */
  public static Region szr = makeRegion("SZR");

  /** The green search zone. */
  public static Region szg = makeRegion("SZG");
  
  /**
   * Receives Wi-Fi parameters from the server program.
   */
  public static void receiveWifiParameters() {
    // Only initialize the parameters if needed
    if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
      return;
    }
    System.out.println("Waiting to receive Wi-Fi parameters.");

    // Connect to server and get the data, catching any errors that might occur
    try (var conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
      /*
       * Connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in.
       */
      wifiParameters = conn.getData();
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }
  
  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   * 
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int getWP(String key) {
    if (wifiParameters != null) {
      return ((BigDecimal) wifiParameters.get(key)).intValue();
    } else {
      return 0;
    }
  }
  
  /** Makes a point given a Wi-Fi parameter prefix. */
  public static Point makePoint(String paramPrefix) {
    return new Point(getWP(paramPrefix + "_x"), getWP(paramPrefix + "_y"));
  }
  
  /** Makes a ramp edge given a Wi-Fi parameter prefix. */
  public static RampEdge makeRampEdge(String paramPrefix) {
    return new RampEdge(makePoint(paramPrefix + "L"), makePoint(paramPrefix + "R"));
  }
  
  /** Makes a region given a Wi-Fi parameter prefix. */
  public static Region makeRegion(String paramPrefix) {
    return new Region(makePoint(paramPrefix + "_LL"), makePoint(paramPrefix + "_UR"));
  }
  
}
