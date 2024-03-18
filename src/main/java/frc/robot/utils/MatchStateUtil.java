package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class MatchStateUtil {
  private boolean realMatch;

  private boolean isBlue;

  private boolean isTeleop;

  /**
   * This class stores the state of the match (the alliance color, whether it's a real match, and whether it's currently teleop)
   *
   * @param realMatch a boolean variable defining whether we are in a real match; this can be
   *     determined by checking if the remaining match time in the init function for the match face
   *     is greater than one second
   * @param isBlue a boolean variable defining whether we are on the blue alliance (true) or the red
   *     alliance (false).
   * @param isTeleop Defines whether the robot is currently in teleoperated mode. Usually false on construction.
   */
  public MatchStateUtil(boolean realMatch, boolean isBlue, boolean isTeleop) {
    this.realMatch = realMatch;
    this.isBlue = isBlue;
    this.isTeleop = isTeleop;
  }

  /**
   * Sets whether it is a real match by checking if the match time is more than 1s, and whether we
   * are blue from the driver station data (defaults to true). User indicates whether it's currently teleop or not.
   */
  public void updateMatchState(boolean isTeleop) {
    this.isTeleop = isTeleop;
    this.realMatch = DriverStation.getMatchTime() > 1.0;
    if (DriverStation.getAlliance().isPresent()) {
      this.isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    } else {
      this.isBlue = true;
    }
  }

  /**
   * Returns true if the match is a "real" (timed, via FMS or practice mode) match. Returns false if
   * it is an untimed match (i.e. the autonomous- or teleop-specific modes) OR if disabled before a real match has begun.
   */
  public boolean isRealMatch() {
    return realMatch;
  }

  /** Returns true if the robot is on the blue alliance */
  public boolean isBlue() {
    return isBlue;
  }

  /** Returns true if the robot is currently in teleoperated mode */
  public boolean isTeleop() {
    return isTeleop;
  }

  /** Returns true if the robot is on the red alliance. */
  public boolean isRed() {
    // Yes, this is the same as !isBlue(), but it's more readable this way
    return !isBlue;
  }

  public void setTeleop(boolean isTeleop) {
    this.isTeleop = isTeleop;
  }
}
