package frc.robot.utils;

public class MatchStateUtil {
  private boolean realMatch;

  private boolean isBlue;

  /**
   * This class stores the state of the match (the alliance color, and whether it's a real match)
   *
   * @param realMatch a boolean variable defining whether we are in a real match; this can be
   *     determined by checking if the remaining match time in the init function for the match face
   *     is greater than one second
   * @param isBlue a boolean variable defining whether we are on the blue alliance (true) or the red
   *     alliance (false). When in a fake match, this variable should be true.
   */
  public MatchStateUtil(boolean realMatch, boolean isBlue) {
    this.realMatch = realMatch;
    this.isBlue = isBlue;
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

  /** Returns true if the robot is on the red alliance. */
  public boolean isRed() {
    // Yes, this is the same as !isBlue(), but it's more readable this way
    return !isBlue;
  }

  public void setRealMatch(boolean realMatch) {
    this.realMatch = realMatch;
  }

  public void setBlue(boolean isBlue) {
    this.isBlue = isBlue;
  }
}
