package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import java.lang.Thread;
import simlejos.hardware.ev3.LocalEV3;

/**
 * Main class of the program.
 * 
 * @author: Vadim Tuchila, Delia Bretan, George Garxenaj, Félix Bédard, Viet Tran, Anandamoyi Saha.
 * 
 *          The Main class of the program represents the entity responsible for launching the application including all
 *          of the necessary threads (odometer and the main threads) used during navigation and robot manipulation. The
 *          main goal fulfilled by the project's software is to make the robot model, which was built in LeoCAD
 *          modelling software and implemented in the Webots simulator, navigate through a Webots simulated world by
 *          avoiding any static or dynamic (moving) obstacles present on its way and to move objects to a corresponding
 *          location while being completely aware of its position in the world. The flow of execution consists of the
 *          robot starting on an island, directing itself towards the main island by passing through a narrow bridge,
 *          then, while avoiding all objects perceived as obstacles on its trajectory, push the appropriate box objects
 *          into the bins located on a small ramp that the robot must be able to locate and climb. Finally, after
 *          pushing all the boxes into the bin, the robot is required to return to its starting position on the initial
 *          island it was spawned on at the beginning of the simulation. Throughout the whole process, concepts such as
 *          odometry, sensor localization (e.g. ultrasonic & light localizations) and obstacle avoidance ought to be
 *          used in order to allow the robot to execute correctly its tasks.
 * 
 */
public class Main {

  /**
   * The number of threads used in the program (main, odometer), other than the one used to perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;

  /** Main entry point. */
  public static void main(String[] args) {
    initialize();

    // Start the odometer thread
    new Thread(odometer).start();

    // TODO Replace these method calls with your own logic
    // LocalEV3.getAudio().beep(); // beeps once
    // wifiExample();
    UltrasonicLocalizer.localize();
    // ObjectDetection.findObjects();
    // ObjectDetection.printMap();
  }

  /**
   * Example using WifiConnection to communicate with a server and receive data concerning the competition such as the
   * starting corner the robot is placed in.<br>
   * 
   * <p>
   * Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi code; you must use the WifiConnection
   * class yourself in your own code as appropriate. In this example, we simply show how to get and process different
   * types of data.<br>
   * 
   * <p>
   * There are two variables you MUST set manually (in Resources.java) before using this code:
   * 
   * <ol>
   * <li>SERVER_IP: The IP address of the computer running the server application. This will be your own laptop, until
   * the beta beta demo or competition where this is the TA or professor's laptop. In that case, set the IP to the
   * default (indicated in Resources).</li>
   * <li>TEAM_NUMBER: your project team number.</li>
   * </ol>
   * 
   * <p>
   * Note: You can disable printing from the Wi-Fi code via ENABLE_DEBUG_WIFI_PRINT.
   * 
   * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur, Olivier St-Martin Cormier
   */
  public static void wifiExample() {
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Red Team: " + redTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Red tunnel footprint, lower left y value: " + tnr.ll.y);

    // Example 3: Compare value
    if (szg.ll.x >= island.ll.x && szg.ll.y >= island.ll.y) {
      System.out.println("The green search zone is on the island.");
    } else {
      System.err.println("The green search zone is in the water!");
    }

    // Example 4: Calculate the area of a region
    System.out.println("The island area is " + island.getWidth() * island.getHeight() + ".");
  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps regularly.
   */
  private static void initialize() {
    // Run a few physics steps to make sure everything is initialized and has settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }

    // We are going to start two threads, so the total number of parties is 2
    setNumberOfParties(NUMBER_OF_THREADS);

    // Does not count as a thread because it is only for physics steps
    new Thread(() -> {
      while (performPhysicsStep()) {
        sleepFor(PHYSICS_STEP_PERIOD);
      }
    }).start();
  }

}
