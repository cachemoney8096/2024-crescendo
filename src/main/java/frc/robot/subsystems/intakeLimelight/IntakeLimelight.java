// Code adapted from team 3005

package frc.robot.subsystems.intakeLimelight;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import java.util.Optional;

/** Limelight for the intake to identify game pieces */
public class IntakeLimelight extends SubsystemBase {
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

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
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
   * Create an IntakeLimelight object
   *
   * @param pitchAngleDegrees pitch angle from normal in degress. Looking straight out is 0, and
   *     increasing as the camera is tilted towards the ceiling.
   * @param heightMeters height of the camera measured from the lens to the ground in meters.
   * @param targetHeightMeters height to the center of the target in meters
   */
  public IntakeLimelight(double pitchAngleDegrees, double heightMeters, double targetHeightMeters) {
    kCameraPitchAngleDegrees = pitchAngleDegrees;
    kCameraHeight = heightMeters;
    kTargetHeight = targetHeightMeters;
    setLimelightValues(
        Constants.limelightLedMode.OFF,
        Constants.limelightCamMode.VISION_PROCESSING,
        Constants.limelightPipeline.NOTE_PIPELINE);
    // setLimelightValues(
    //     Constants.limelightLedMode.OFF,
    //     Constants.limelightCamMode.DRIVER_CAMERA,
    //     Constants.limelightPipeline.TAG_PIPELINE);

    m_simDevice = SimDevice.create("limelight-intake");
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
    if (tagIdRounded == 11
        || tagIdRounded == 12
        || tagIdRounded == 13
        || tagIdRounded == 14
        || tagIdRounded == 15
        || tagIdRounded == 16) {
      return true;
    } else {
      return false;
    }
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

  public Optional<Transform2d> checkForTag() {
    if (getValidTarget() != 1) {
      return Optional.empty();
    }

    Pose3d cameraToTag =
        LimelightHelpers.getTargetPoseCameraSpace(IntakeLimelightConstants.INTAKE_LIMELIGHT_NAME);
    Transform2d robotToTag =
        new Transform2d(
            new Translation2d(cameraToTag.getZ(), -cameraToTag.getX()),
            Rotation2d.fromRadians(-cameraToTag.getRotation().getY()));

    System.out.println("robotToTag: " + robotToTag);
    System.out.println("trap offset: " + IntakeLimelightCal.TRAP_OFFSET);
    System.out.println("sum: " + robotToTag.plus(IntakeLimelightCal.TRAP_OFFSET));

    return Optional.of(robotToTag.plus(IntakeLimelightCal.TRAP_OFFSET));
  }

  public double getLatencySeconds() {
    return (LimelightHelpers.getLatency_Capture(IntakeLimelightConstants.INTAKE_LIMELIGHT_NAME)
            + LimelightHelpers.getLatency_Pipeline(IntakeLimelightConstants.INTAKE_LIMELIGHT_NAME))
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
    if (getValidTarget() > 0) {
      return true;
    } else {
      return false;
    }
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

  public class NoteDetection {
    public double latencySec;

    /** Distance from camera */
    public double distanceMeters;

    /** CCW Yaw Angle */
    public double yawAngleDeg;

    public NoteDetection(double latencySec, double distanceMeters, double yawAngleDeg) {
      this.latencySec = latencySec;
      this.distanceMeters = distanceMeters;
      this.yawAngleDeg = yawAngleDeg;
    }
  }

  /**
   * Looks for a note
   *
   * @return If empty, no note detected. First value is note yaw in degrees (ccw), second value is
   *     distance in meters.
   */
  public Optional<NoteDetection> getNotePos() {
    // TODO filter low confidence detection in LL dashboard, hopefully
    if (getPipeline() != Constants.limelightPipeline.NOTE_PIPELINE) {
      setPipeline(Constants.limelightPipeline.NOTE_PIPELINE);
    }

    LimelightHelpers.LimelightResults llresults =
        LimelightHelpers.getLatestResults(IntakeLimelightConstants.INTAKE_LIMELIGHT_NAME);
    LimelightTarget_Detector[] targets_Detector = llresults.targetingResults.targets_Detector;

    if (targets_Detector.length == 0) {
      // System.out.println("Didn't see note");
      return Optional.empty();
    }

    LimelightTarget_Detector lowestDetection = targets_Detector[0];
    for (LimelightTarget_Detector detection : targets_Detector) {
      if (detection.className == "note") {
        if (detection.ty_pixels > lowestDetection.ty_pixels) { // lower in image = greater y value
          lowestDetection = detection;
        }
      }
    }

    double angleLimelightToNoteDegrees =
        lowestDetection.ty - IntakeLimelightConstants.INTAKE_LIMELIGHT_PITCH_DEGREES;
    double noteDistanceMeters =
        (IntakeLimelightConstants.INTAKE_LIMELIGHT_HEIGHT_METERS
                - Units.inchesToMeters(Constants.NOTE_HEIGHT_INCHES / 2))
            / Math.tan(Units.degreesToRadians(angleLimelightToNoteDegrees));

    double yawAngleXDegrees = lowestDetection.tx;
    double adjustedYawAngleDegrees = yawAngleXDegrees + IntakeLimelightCal.LIMELIGHT_YAW_DEGREES;
    return Optional.of(
        new NoteDetection(getLatency(), -noteDistanceMeters, -adjustedYawAngleDegrees));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Latency", () -> getLatency(), null);
    builder.addDoubleProperty("Tx", () -> getOffSetX(), null);
    builder.addDoubleProperty("Ty", () -> getOffSetY(), null);
    // builder.addDoubleProperty("Note Latency (sec)", () -> {
    //   var maybeNotePos = getNotePos();
    //   if (maybeNotePos.isEmpty())
    //   {
    //     return 0.0;
    //   }
    //   return maybeNotePos.get().latencySec;
    // }, null);
    builder.addDoubleProperty("Note Distance (m)", () -> {
      var maybeNotePos = getNotePos();
      if (maybeNotePos.isEmpty())
      {
        return 0.0;
      }
      return maybeNotePos.get().distanceMeters;
    }, null);
    // builder.addDoubleProperty("Note Angle (deg)", () -> {
    //   var maybeNotePos = getNotePos();
    //   if (maybeNotePos.isEmpty())
    //   {
    //     return 0.0;
    //   }
    //   return maybeNotePos.get().yawAngleDeg;
    // }, null);

    builder.addBooleanProperty("Valid Target", () -> isValidTarget(), null);
    builder.addBooleanProperty("Connected", () -> CheckConnection(), null);
  }
}
