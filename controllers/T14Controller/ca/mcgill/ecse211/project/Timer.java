package ca.mcgill.ecse211.project;
//importing necessary libraries
import static ca.mcgill.ecse211.project.Resources.*;

/*
 * Class that ensures that simulation runs under given time constraints, and tracks the timeframe throughout which robot
 * conducts operations with respect of team's flowchart and each part of the robot's trajectory
 */

public class Timer implements Runnable {

  // initializations
  public static Timer countdown = new Timer();
  private int seconds = 300;

  /** Creates a Timer object. Use this method to obtain an instance of Timer. */
  public static synchronized Timer getTimer() {
    if (countdown == null) {
      countdown = new Timer();
    }
    return countdown;
  }

  @Override
  /** Will count down from the given 5 minute time constraint, to be used when the main method starts its run */
  public void run() {
    while (seconds > 0) {
      try {
        // countdown the seconds
        seconds--;
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        System.out.println("Timer error");
      }
      // at a minute left, robot drops what it is doing and thread prompts it to return to starting zone.
      if (seconds == 60) {
        Driver.stopMotors();
        Driver.setSpeed(FORWARD_SPEED);
        Navigation.travelBackAcrossTunnel();
      }
    }
    //simulation end
    if (seconds == 0) {  
      Driver.stopMotors();
      System.out.println("Simulation is over");
    }
  }

  /* Getter method to return time left in the simulation*/
  public  int getTime() {
   
    return seconds;
  }
}