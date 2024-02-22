// Code from team 3005

package frc.robot.subsystems.shooterLimelight;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.Optional;

/** Limelight for the shooter to identify game pieces */
public class ShooterLimelight extends SubsystemBase {
  private final double kCameraPitchAngleDegrees;
  private final double kCameraHeight;
  private final double kTargetHeight;
  private final double kImageCaptureLatency = 11.0;

  // In 2023, 960x720. In 2024, may change because of potentially different FPS requirements
  private final double RESOLUTION_X = Constants.PLACEHOLDER_DOUBLE;
  private final double RESOLUTION_Y = Constants.PLACEHOLDER_DOUBLE;

  // Simulation functions
  private SimDevice m_simDevice;
  private SimDouble m_targetArea;
  private SimDouble m_skew;
  private SimDouble m_latency;
  private SimDouble m_tx;
  private SimDouble m_ty;
  private SimBoolean m_valid;

  // NT published variables when using translation api
  private double m_lastDistance = 0.0;
  private double m_lastX = 0.0;
  private double m_lastY = 0.0;

  NetworkTable table =
      NetworkTableInstance.getDefault().getTable(ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME);
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tclass = table.getEntry("tclass");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");

  // AprilTag detection
  private Optional<Transform2d> robotToScoringLocation = Optional.empty();

  /**
   * Create an ShooterLimelight object
   *
   * @param pitchAngleDegrees pitch angle from normal in degress. Looking straight out is 0, and
   *     increasing as the camera is tilted towards the ceiling.
   * @param heightMeters height of the camera measured from the lens to the ground in meters.
   * @param targetHeightMeters height to the center of the target in meters
   */
  public ShooterLimelight(
      double pitchAngleDegrees, double heightMeters, double targetHeightMeters) {
    kCameraPitchAngleDegrees = pitchAngleDegrees;
    kCameraHeight = heightMeters;
    kTargetHeight = targetHeightMeters;
    setLimelightValues(
        Constants.limelightLedMode.OFF,
        Constants.limelightCamMode.VISION_PROCESSING,
        Constants.limelightPipeline.TAG_PIPELINE);

    m_simDevice = SimDevice.create(ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME);
    if (m_simDevice != null) {
      m_targetArea = m_simDevice.createDouble("Target Area", Direction.kBidir, 0.0);
      m_skew = m_simDevice.createDouble("Skew", Direction.kBidir, 0.0);
      m_latency = m_simDevice.createDouble("Latency", Direction.kBidir, 0.0);
      m_tx = m_simDevice.createDouble("Tx", Direction.kBidir, 0.0);
      m_ty = m_simDevice.createDouble("Ty", Direction.kBidir, 0.0);
      m_valid = m_simDevice.createBoolean("Valid", Direction.kBidir, false);
    }
  }

  private static Transform2d getBotFromTarget(Pose3d botPoseTargetSpace) {
    /**
     * Target space: 3d Cartesian Coordinate System with (0,0,0) at the center of the target.
     *
     * <p>X+ → Pointing to the right of the target (If you are looking at the target)
     *
     * <p>Y+ → Pointing downward
     *
     * <p>Z+ → Pointing out of the target (orthogonal to target’s plane).
     */

    /**
     * We convert to 2d target space: X+ -> Out of the target Y+ -> Pointing to the right of the
     * target (If you are looking at the target) This means positive yaw is based on Z+ being up
     */
    Translation2d translation =
        new Translation2d(botPoseTargetSpace.getZ(), botPoseTargetSpace.getX());
    Rotation2d rot = Rotation2d.fromDegrees(-botPoseTargetSpace.getRotation().getY());

    System.out.println("Tag at " + -botPoseTargetSpace.getRotation().getY() + " deg");
    return new Transform2d(translation, rot);
  }

  private static boolean validScoringTag(double tagId) {
    long tagIdRounded = Math.round(tagId);
    return (tagIdRounded == 3 || tagIdRounded == 4 || tagIdRounded == 7 || tagIdRounded == 8);
  }

  public static int chooseTag(LimelightTarget_Fiducial[] targets) {
    int numTags = targets.length;
    double minDistMeters = Double.MAX_VALUE;
    int bestTag = 0;
    for (int tagIndex = 0; tagIndex < numTags; tagIndex++) {
      LimelightTarget_Fiducial target = targets[tagIndex];
      if (!validScoringTag(target.fiducialID)) {
        continue;
      }
      double targetDistance = target.getTargetPose_RobotSpace().getTranslation().getNorm();
      if (targetDistance < minDistMeters) {
        minDistMeters = targetDistance;
        bestTag = tagIndex;
      }
    }
    return bestTag;
  }

  private Transform2d getRobotToScoringLocation(Pose3d targetPoseRobotSpace) {
    Transform2d targetFromBot = getBotFromTarget(targetPoseRobotSpace);
    return targetFromBot;
  }

  public Optional<Transform2d> getRobotToScoringLocation() {
    return robotToScoringLocation;
  }

  public Optional<Transform2d> checkForTag() {
    if (!LimelightHelpers.getTV(ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME)) {
      robotToScoringLocation = Optional.empty();
      return Optional.empty();
    }
    if (!validScoringTag(
        LimelightHelpers.getFiducialID(ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME))) {
      robotToScoringLocation = Optional.empty();
      return Optional.empty();
    }
    robotToScoringLocation =
        Optional.of(
            getRobotToScoringLocation(
                LimelightHelpers.getTargetPose3d_RobotSpace(
                    ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME)));
    return robotToScoringLocation;
  }

  public double getLatencySeconds() {
    return (LimelightHelpers.getLatency_Capture(ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME)
            + LimelightHelpers.getLatency_Pipeline(
                ShooterLimelightConstants.SHOOTER_LIMELIGHT_NAME))
        / 1000.0;
  }

  /**
   * Sets the led mode.
   *
   * @param mode LED operating mode.
   */
  public void setLedMode(Constants.limelightLedMode mode) {
    table.getEntry("ledMode").setNumber(mode.ordinal());
  }

  /**
   * Sets the camera operating mode.
   *
   * @param mode Camera operating mode.
   */
  public void setCamMode(Constants.limelightCamMode mode) {
    table.getEntry("camMode").setNumber(mode.ordinal());
  }

  /**
   * Sets the vision thresholding pipeline.
   *
   * @param line Pipeline index
   */
  public void setPipeline(Constants.limelightPipeline line) {
    table.getEntry("pipeline").setNumber(line.ordinal());
  }

  /**
   * Gets the current vision pipeline
   *
   * @return pipeline
   */
  public Constants.limelightPipeline getPipeline() {
    return Constants.limelightPipeline.values()[(int) table.getEntry("getpipe").getDouble(0)];
  }

  public void setLimelightValues(
      Constants.limelightLedMode ledMode,
      Constants.limelightCamMode camMode,
      Constants.limelightPipeline line) {
    setLedMode(ledMode);
    setCamMode(camMode);
    setPipeline(line);

    SmartDashboard.putString("LED Mode", ledMode.name());
    SmartDashboard.putString("Cam Mode", camMode.name());
    SmartDashboard.putString("Pipeline", line.name());
  }

  /**
   * @return validObject - Whether the limelight has any valid targets (0 or 1)
   */
  public double getValidTarget() {
    if (m_simDevice != null) {
      return m_valid.get() ? 1 : 0;
    }
    return table.getEntry("tv").getDouble(-1);
  }

  /**
   * @return xOffSet - Horizontal Offset(Left to Right) From Crosshair To Target (LL2: -29.8 to 29.8
   *     degrees)
   */
  public double getOffSetX() {
    if (m_simDevice != null) {
      return m_tx.get();
    }
    return table.getEntry("tx").getDouble(0.0);
  }

  /**
   * @return yOffSett - Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees
   *     | LL2: -24.85 to 24.85 degrees)
   */
  public double getOffSetY() {
    if (m_simDevice != null) {
      return m_ty.get();
    }
    return table.getEntry("ty").getDouble(0.0);
  }

  /**
   * @return targetArea - Target Area (0% of image to 100% of image)
   */
  public double getTargetArea() {
    if (m_simDevice != null) {
      return m_targetArea.get();
    }
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * @return skew - Skew or rotation (-90 degrees to 0 degrees)
   */
  public double getSkew() {
    if (m_simDevice != null) {
      return m_skew.get();
    }
    return table.getEntry("ts").getDouble(0.0);
  }

  /**
   * @return latency - The pipeline’s latency contribution in seconds. Add at least 11ms for image
   *     capture latency.
   */
  public double getLatency() {
    if (m_simDevice != null) {
      return m_latency.get() + kImageCaptureLatency;
    }
    return (table.getEntry("tl").getDouble(0.0) + kImageCaptureLatency) / 1e3;
  }

  /**
   * Get the timestamp of the last update to the network table. This can be used to get a better
   * estimate of the total latency. That is (lastUpdate - latency).
   *
   * @return timestamp of the last update to the latency update in seconds.
   */
  public double getLastTimestamp() {
    if (m_simDevice != null) {
      return Timer.getFPGATimestamp();
    }
    return table.getEntry("tl").getLastChange() / 1e6;
  }

  /**
   * @return true is limelight has made a target else false
   */
  public boolean isValidTarget() {
    return getValidTarget() > 0;
  }

  /**
   * @return true as long as limelight does not have value of -1
   */
  public boolean CheckConnection() {
    return getValidTarget() != -1.0;
  }

  /**
   * Get a 2d translation from the camera to the target, including normalization to handle the
   * effects of angle to target. See the below discussion.
   * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7
   *
   * @parm targetHeightMeters use a target height other than what is provided in constructor.
   * @return Translation2d from the camera to the target. This is camera relative, x is out of the
   *     camera and positive y is to the left from the camera's point of view.
   */
  public Translation2d getTargetTranslation(double targetHeightMeters) {
    /**
     * This function uses the limelight coordinates until the end of the function. That is, the x
     * axis is left/right, horizontal to the field. The y axis is 'up/down' normal to the field
     * surface, and z is away from the camera, horizontal to the field surface.
     *
     * <p>The output coordinates are only a translation in x/y to the target, where x is out of the
     * camera, horizontal to the field, and y is positive to the left from the perspective of the
     * camera.
     */
    double diff_height = targetHeightMeters - kCameraHeight;

    // x is the "left/right" to the target from the camera
    double x = Math.tan(Math.toRadians(getOffSetX()));

    // y is the 'up/down' to the target from the center of the camera
    double y = Math.tan(Math.toRadians(getOffSetY()));

    // z is straight out of the camera
    double z = 1.0;
    double length = Math.sqrt(x * x + y * y + z * z);

    // Normalized vector components
    double x_norm = x / length;
    double y_norm = y / length;
    double z_norm = z / length;

    // Rotate vector by camera degrees around camera x axis
    Translation2d zy_translation =
        new Translation2d(z_norm, y_norm)
            .rotateBy(Rotation2d.fromDegrees(kCameraPitchAngleDegrees));
    z_norm = zy_translation.getX();
    y_norm = zy_translation.getY();

    // prevent divide by zero
    if (Math.abs(y_norm) < 1e-4) {
      y_norm = Math.signum(y_norm) * 1e-4;
    }

    /**
     * Find the intersection between the target in space, and the vector pointing to the target
     *
     * <p>This becomes a vector [x_norm, y_norm, z_norm] * d = [x_target, diff_height, z_target]
     *
     * <p>x_norm * d = x_target y_norm * d = diff_height z_target * d = z_target
     */
    double scaling = diff_height / y_norm;
    double x_target = scaling * x_norm;
    double z_target = scaling * z_norm;
    double distance = Math.hypot(z_target, x_target);

    // Convert camera coordinates to our conventions
    Translation2d result = new Translation2d(distance, new Rotation2d(z_target, -x_target));

    // For NT so this function doesn't need to be called multiple times
    m_lastDistance = distance;
    m_lastX = result.getX();
    m_lastY = result.getY();

    return result;
  }

  /**
   * Get a 2d translation from the camera to the target, including normalization to handle the
   * effects of angle to target. See the below discussion.
   * https://www.chiefdelphi.com/t/what-does-limelight-skew-actually-measure/381167/7
   *
   * @return Translation2d from the camera to the target. This is camera relative, x is out of the
   *     camera and positive y is to the left from the camera's point of view.
   */
  public Translation2d getTargetTranslation() {
    return getTargetTranslation(kTargetHeight);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Area", () -> getTargetArea(), null);
    builder.addDoubleProperty("Skew", () -> getSkew(), null);
    builder.addDoubleProperty("Latency", () -> getLatency(), null);
    builder.addDoubleProperty("Tx", () -> getOffSetX(), null);
    builder.addDoubleProperty("Ty", () -> getOffSetY(), null);
    builder.addBooleanProperty("Valid Target", () -> isValidTarget(), null);
  }
}
