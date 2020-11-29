package ca.mcgill.ecse211.project;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Stopwatch implements Runnable {

  /** The current system time */
  private static double currentTime;

  /** The time the program was started */
  private static double startTime;

  /** The time that has passed */
  private static double timeElapsed;

  /** Fair lock for concurrent writing. */
  private static Lock lock = new ReentrantLock(true);

  /** The singleton stopwatch instance. */
  private static Stopwatch stopwatch;

  // This method indicates how the stopwatch will run
  @Override
  public void run() {
    while (timeElapsed < 300) {
      currentTime = TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.SECONDS);
      timeElapsed = currentTime - startTime;
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      System.out.println(timeElapsed);
    }
      
    //Travel
  }

  /** Returns the Stopwatch Object. Use this method to obtain an instance of Stopwatch. */
  public static synchronized Stopwatch getStopwatch() {
    if (stopwatch == null) {
      startTime = TimeUnit.SECONDS.convert(System.nanoTime(), TimeUnit.SECONDS);
      stopwatch = new Stopwatch();
    }
    return stopwatch;
  }

  
  /** @return the time that has passed since the start of the program*/
  public double getTime() {
    double timePassed;
    lock.lock();
    try {
      timePassed = timeElapsed;
    }   finally {
      lock.unlock();
    }
    
    return timePassed;
  }

}
