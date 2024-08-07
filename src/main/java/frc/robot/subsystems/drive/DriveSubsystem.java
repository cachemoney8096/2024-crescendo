package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.shooter.ShooterCal;
import frc.robot.utils.GeometryUtils;
import frc.robot.utils.MatchStateUtil;
import frc.robot.utils.PoseBuffer;
import java.util.List;
import java.util.Optional;

public class DriveSubsystem extends SubsystemBase {
  private double targetHeadingDegrees;

  double tagTargetHeading = 0.0;

  public void setTargetHeadingDegrees(double targetHeadingDegrees) {
    this.targetHeadingDegrees = targetHeadingDegrees;
    tagTargetHeading = targetHeadingDegrees;
  }

  // Create SwerveModules
  public final SwerveModule frontLeft =
      new SwerveModule(
          RobotMap.FRONT_LEFT_DRIVING_CAN_ID,
          RobotMap.FRONT_LEFT_TURNING_CAN_ID,
          DriveCal.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule frontRight =
      new SwerveModule(
          RobotMap.FRONT_RIGHT_DRIVING_CAN_ID,
          RobotMap.FRONT_RIGHT_TURNING_CAN_ID,
          DriveCal.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearLeft =
      new SwerveModule(
          RobotMap.REAR_LEFT_DRIVING_CAN_ID,
          RobotMap.REAR_LEFT_TURNING_CAN_ID,
          DriveCal.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

  public final SwerveModule rearRight =
      new SwerveModule(
          RobotMap.REAR_RIGHT_DRIVING_CAN_ID,
          RobotMap.REAR_RIGHT_TURNING_CAN_ID,
          DriveCal.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

  private final Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
  private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  public Optional<Pose2d> targetPose = Optional.empty();

  public PoseBuffer poseBuffer = new PoseBuffer();

  double KeepHeadingPID = 0.0;
  double KeepHeadingFF = 0.0;

  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.DRIVE_KINEMATICS,
          Rotation2d.fromDegrees(Constants.PLACEHOLDER_DOUBLE),
          getModulePositions());

  /** Multiplier for drive speed, does not affect trajectory following */
  public double throttleMultiplier = 1.0;

  private double rotControllerInput = 0.0;

  /** Provides info on our alliance color and whether this is a real match. */
  public MatchStateUtil matchState;

  /**
   * Interpolation map storing rotational velocities (deg/s) as keys and the amount the robot
   * overshoots at each velocity (deg) as values
   */
  private InterpolatingDoubleTreeMap yawOffsetMap;

  /**
   * Interpolation map to convert drive velocities (m/s) as keys and a value in [0,1] for the
   * keepheading PID multiplier as values
   */
  private InterpolatingDoubleTreeMap velocityToMultiplierMap;

  public DriveSubsystem(MatchStateUtil matchState) {
    intializeGyro();
    this.matchState = matchState;

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getCurrentChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setOutputRobotRelativeSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            DriveCal.PATH_TRANSLATION_CONTROLLER, // Translation PID constants
            DriveCal.PATH_ROTATION_CONTROLLER, // Rotation PID constants
            DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND, // Max module speed, in m/s
            DriveConstants
                .DRIVE_BASE_RADIUS_METERS, // Drive base radius in meters. Distance from robot
            // center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    yawOffsetMap = new InterpolatingDoubleTreeMap();
    yawOffsetMap.put(0.0, 0.0);
    yawOffsetMap.put(120.0, 5.0);
    yawOffsetMap.put(167.0, 22.0);
    yawOffsetMap.put(330.0, 50.0);

    velocityToMultiplierMap = new InterpolatingDoubleTreeMap();
    velocityToMultiplierMap.put(0.0, DriveCal.MIN_ROTATE_TO_TARGET_PID_OUTPUT);
    velocityToMultiplierMap.put(DriveConstants.MAX_SPEED_METERS_PER_SECOND, 1.0);

    SmartDashboard.putNumber("Norm Velocity (mps)", 0);
    SmartDashboard.putNumber("Velocity PID multiplier", 0);
  }

  public void intializeGyro() {
    // Reset to defaults
    Pigeon2Configurator cfg = gyro.getConfigurator();
    Pigeon2Configuration blankGyroConfiguration = new Pigeon2Configuration();
    cfg.apply(blankGyroConfiguration);
    final double fastUpdateFrequencyHz = 50.0; // todo update for better odometry
    gyro.getYaw().setUpdateFrequency(fastUpdateFrequencyHz);
    gyro.getAngularVelocityZWorld().setUpdateFrequency(fastUpdateFrequencyHz);

    // Set mount pose
    MountPoseConfigs gyroConfig = new MountPoseConfigs();
    gyroConfig.MountPosePitch = 0;
    gyroConfig.MountPoseRoll = 0;
    gyroConfig.MountPoseYaw = 90;
    cfg.apply(gyroConfig);

    // Reset position to zero, this may be overwritten by a path at the start of auto
    gyro.reset();
    Timer.delay(0.1);
    gyro.reset();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    frontLeft.periodic();
    frontRight.periodic();
    rearLeft.periodic();
    rearRight.periodic();
    odometry.update(Rotation2d.fromDegrees(gyro.getYaw().getValue()), getModulePositions());
    poseBuffer.pushToBuffer(getPose(), Timer.getFPGATimestamp());
    // TODO: put this in a thread that loops faster
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      rearLeft.getPosition(),
      rearRight.getPosition()
    };
  }

  public void burnFlashSparks() {
    frontLeft.burnFlashSparks();
    frontRight.burnFlashSparks();
    rearLeft.burnFlashSparks();
    rearRight.burnFlashSparks();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    ChassisSpeeds chassisSpeeds =
        DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState());
    return chassisSpeeds;
  }

  /**
   * applies the robot-relative output speeds of the FollowPathHolonomic Command adapted from
   * drive() (but doesn't need the code to turn xSpeed, ySpeed, rot, fieldRelative into a
   * ChassisSpeeds object)
   */
  public void setOutputRobotRelativeSpeeds(ChassisSpeeds desiredChassisSpeeds) {

    ChassisSpeeds correctedDesiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    lastSetChassisSpeeds = correctedDesiredChassisSpeeds;
    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public void resetYawToAngle(double yawDeg) {
    double curYawDeg = gyro.getYaw().getValue();
    double offsetToTargetDeg = targetHeadingDegrees - curYawDeg;
    gyro.setYaw(yawDeg);
    Pose2d curPose = getPose();
    Pose2d resetPose = new Pose2d(curPose.getTranslation(), Rotation2d.fromDegrees(yawDeg));
    odometry.resetPosition(Rotation2d.fromDegrees(yawDeg), getModulePositions(), resetPose);
    targetHeadingDegrees = yawDeg + offsetToTargetDeg;
  }

  public void resetYaw() {
    resetYawToAngle(matchState.isBlue() ? 0 : 180);
  }

  //TODO 718AIM Test rezeroing
  public void resetOdometryToCenterSubwoofer() {
    double odometryXMeters = matchState.isBlue() ? 1.168 : 15.429;
    double odometryYMeters = matchState.isBlue() ? 5.467 : 5.422;
    double curYawDeg = gyro.getYaw().getValue();
    double offsetToTargetDeg = targetHeadingDegrees - curYawDeg;
    double yawDeg = matchState.isBlue() ? 0 : 180;
    gyro.setYaw(yawDeg);
    Pose2d resetPose = new Pose2d(new Translation2d(odometryXMeters, odometryYMeters), Rotation2d.fromDegrees(yawDeg));
    odometry.resetPosition(Rotation2d.fromDegrees(yawDeg), getModulePositions(), resetPose);
    targetHeadingDegrees = yawDeg + offsetToTargetDeg;
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    // TODO test arbitrarily making this larger to see if it helps
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /** Keep modules in current position, don't drive */
  public void setNoMove() {
    Rotation2d frontLeftCurrRot = frontLeft.getPosition().angle;
    Rotation2d frontRightCurrRot = frontRight.getPosition().angle;
    Rotation2d rearLeftCurrRot = rearLeft.getPosition().angle;
    Rotation2d rearRightCurrRot = rearRight.getPosition().angle;
    frontLeft.setDesiredState(new SwerveModuleState(0, frontLeftCurrRot), true);
    frontRight.setDesiredState(new SwerveModuleState(0, frontRightCurrRot), true);
    rearLeft.setDesiredState(new SwerveModuleState(0, rearLeftCurrRot), true);
    rearRight.setDesiredState(new SwerveModuleState(0, rearRightCurrRot), true);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Desired speed of the robot in the x direction (forward), [-1,1].
   * @param ySpeed Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
      setNoMove();
      return;
    }
    xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    rot *= DriveConstants.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

    xSpeed *= throttleMultiplier;
    ySpeed *= throttleMultiplier;
    rot *= throttleMultiplier;

    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw().getValue()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    lastSetChassisSpeeds = desiredChassisSpeeds;

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0], false);
    frontRight.setDesiredState(desiredStates[1], false);
    rearLeft.setDesiredState(desiredStates[2], false);
    rearRight.setDesiredState(desiredStates[3], false);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetDriveEncoder();
    rearLeft.resetDriveEncoder();
    frontRight.resetDriveEncoder();
    rearRight.resetDriveEncoder();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValue()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  /**
   * Keeps the heading of the robot when the driver is not turning, by using PID to keep the
   * distance between the actual heading and the last intended heading to 0.
   *
   * @param x Desired speed of the robot in the x direction (forward), [-1,1].
   * @param y Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void keepHeading(double x, double y, boolean fieldRelative) {
    double currentHeadingDegrees = getHeadingDegrees();
    double headingDifferenceDegrees = currentHeadingDegrees - targetHeadingDegrees;
    double offsetHeadingDegrees = MathUtil.inputModulus(headingDifferenceDegrees, -180, 180);

    double pidRotation =
        DriveCal.ROTATE_TO_TARGET_PID_CONTROLLER.calculate(offsetHeadingDegrees, 0.0);
    double ffRotation = Math.signum(offsetHeadingDegrees) * DriveCal.ROTATE_TO_TARGET_FF;

    double normVelocity =
        new Translation2d(
                lastSetChassisSpeeds.vxMetersPerSecond, lastSetChassisSpeeds.vyMetersPerSecond)
            .getNorm();

    SmartDashboard.putNumber("Norm Velocity (mps)", normVelocity);

    double velocityMultiplier = velocityToMultiplierMap.get(normVelocity);

    SmartDashboard.putNumber("Velocity PID multiplier", velocityMultiplier);

    pidRotation *= velocityMultiplier;

    KeepHeadingPID = pidRotation;
    KeepHeadingFF = ffRotation;

    double desiredRotation = pidRotation - ffRotation;

    if (Math.abs(desiredRotation) < DriveCal.ROTATION_DEADBAND_THRESHOLD) {
      desiredRotation = 0;
    }

    desiredRotation = MathUtil.clamp(desiredRotation, -1.0, 1.0);

    drive(x, y, desiredRotation, fieldRelative);
  }

  /**
   * Rotates to a specific angle from the D-pad buttons
   *
   * @param povAngleDeg Angle of D-pad control
   */
  public int convertCardinalDirections(int povAngleDeg) {
    // TODO: figure out what angles the robot should turn to
    if (povAngleDeg == 270) {
      povAngleDeg += 0;
    } else if (povAngleDeg == 90) {
      povAngleDeg -= 0;
    }
    // targetHeadingDegrees is counterclockwise so need to flip povAngle
    povAngleDeg = 360 - povAngleDeg;
    return povAngleDeg;
  }

  /**
   * Determines whether to rotate according to input or to run the keep heading code, by checking if
   * the (already deadbanded) rotation input is equal to 0.
   *
   * @param x Desired speed of the robot in the x direction (forward), [-1,1].
   * @param y Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param cardinalAngleDeg Get the angle in degrees of the D-pad (clockwise, -1 means POV not
   *     pressed).
   */
  public void rotateOrKeepHeading(
      double x, double y, double rot, boolean fieldRelative, int cardinalAngleDeg) {
    rotControllerInput = rot;
    if (cardinalAngleDeg != -1) {
      targetHeadingDegrees = convertCardinalDirections(cardinalAngleDeg);
      keepHeading(x, y, fieldRelative);
    } else if (rot == 0) {
      keepHeading(x, y, fieldRelative);
    } else {
      targetHeadingDegrees =
          getHeadingDegrees()
              + calculateYawOffsetDeg(
                  Units.radiansToDegrees(lastSetChassisSpeeds.omegaRadiansPerSecond));
      drive(x, y, rot, fieldRelative);
    }
  }

  public Pigeon2 getGyro() {
    return gyro;
  }

  /** adapted from last year's version, which was taken from Github */
  public Command followTrajectoryCommand(PathPlannerPath path, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                this.resetOdometry(path.getPreviewStartingHolonomicPose());
              }
            }),
        new FollowPathHolonomic( // old method from last year: PPSwerveControllerCommand
            path,
            this::getPose, // pose supplier
            this::getCurrentChassisSpeeds, // robot relative chassis speeds supplier
            this::setOutputRobotRelativeSpeeds, // robot relative chassis speeds consumer
            DriveCal.PATH_TRANSLATION_CONTROLLER,
            DriveCal.PATH_ROTATION_CONTROLLER,
            DriveConstants.DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND, // maxModuleSpeed
            DriveConstants.DRIVE_BASE_RADIUS_METERS,
            new ReplanningConfig(), // TODO: creates a path replanning configuration with the
            // default config
            () -> {
              return false;
            }, // boolean supplier for shouldFlipPath
            this),
        new InstantCommand(
            () -> {
              targetHeadingDegrees = getHeadingDegrees();
            }),
        new PrintCommand("Finished a trajectory"));
  }
  /** Returns a past pose, using the PoseBuffer */
  public Pose2d getPastBufferedPose(double latencySec) {
    Optional<Pose2d> p = poseBuffer.getPoseAtTimestamp(Timer.getFPGATimestamp() - latencySec);
    if (!p.isPresent()) {
      return extrapolatePastPoseBasedOnVelocity(Timer.getFPGATimestamp() - latencySec);
    }
    return p.get();
  }
  /** Returns a past pose, given a latency adjustment */
  public Pose2d extrapolatePastPoseBasedOnVelocity(double latencySec) {
    Pose2d curPose = getPose();
    double latencyAdjustmentSec = 0.00;
    latencySec += latencyAdjustmentSec;
    Transform2d pastTransform =
        new Transform2d(
            new Translation2d(
                -lastSetChassisSpeeds.vxMetersPerSecond * latencySec,
                -lastSetChassisSpeeds.vyMetersPerSecond * latencySec),
            Rotation2d.fromRadians(lastSetChassisSpeeds.omegaRadiansPerSecond * latencySec)
                .unaryMinus());
    Pose2d pastPose = curPose.plus(pastTransform);
    return pastPose;
  }

  public void setLimelightTargetFromTransform(
      Transform2d transform, double latencySec, boolean usingFrontLimelight) {
    // Transform is to get the limelight to the correct location, not to get the
    // robot
    // Here we correct for that
    Transform2d flipTransform =
        new Transform2d(
            new Translation2d(
                usingFrontLimelight ? (transform.getX()) : (-transform.getX()),
                usingFrontLimelight ? (transform.getY()) : (-transform.getY())),
            transform.getRotation());

    double latencyAdjustmentSec = 0.00;

    Pose2d curPose = getPose();
    Pose2d pastPose = getPastBufferedPose(latencyAdjustmentSec);
    // TODO: see if we can get this working with the real latencySec

    final boolean useLatencyAdjustment = false;

    targetPose =
        useLatencyAdjustment
            ? Optional.of(pastPose.plus(flipTransform))
            : Optional.of(curPose.plus(flipTransform));
  }

  public PathPlannerPath pathToPoint(Pose2d finalPose, double finalSpeedMetersPerSec) {
    Pose2d curPose = getPose();
    Transform2d finalTransform =
        new Transform2d(finalPose.getTranslation(), finalPose.getRotation());
    System.out.println(
        "Trajectory Transform: " + finalTransform.getX() + " " + finalTransform.getY());

    Rotation2d finalHolonomicRotation = finalPose.getRotation();

    List<Translation2d> bezierTranslations = PathPlannerPath.bezierFromPoses(curPose, finalPose);
    PathPlannerPath path =
        new PathPlannerPath(
                bezierTranslations,
                new PathConstraints(
                    DriveCal.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
                    DriveCal.MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ,
                    DriveCal.MEDIUM_ANGULAR_SPEED_RAD_PER_SEC,
                    DriveCal.MEDIUM_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ),
                new GoalEndState(finalSpeedMetersPerSec, finalHolonomicRotation))
            .replan(curPose, lastSetChassisSpeeds);

    return path;
  }

  public Optional<PathPlannerPath> poseToPath() {
    Pose2d curPose = getPose();

    System.out.println("Acquired target? " + targetPose.isPresent());
    if (!targetPose.isPresent()) {
      return Optional.empty();
    }

    Pose2d finalPose = targetPose.get();
    List<Translation2d> bezierTranslations = PathPlannerPath.bezierFromPoses(curPose, finalPose);

    Rotation2d finalHolonomicRotation =
        Rotation2d.fromDegrees(0); // TODO we may need to question this

    PathPlannerPath path =
        new PathPlannerPath(
            bezierTranslations,
            new PathConstraints(
                DriveCal.MEDIUM_LINEAR_SPEED_METERS_PER_SEC,
                DriveCal.MEDIUM_LINEAR_ACCELERATION_METERS_PER_SEC_SQ,
                DriveCal.MEDIUM_ANGULAR_SPEED_RAD_PER_SEC,
                DriveCal.MEDIUM_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ),
            new GoalEndState(0.0, finalHolonomicRotation));

    return Optional.of(path);
  }

  /** Driving inputs will get multiplied by the throttle value, so it should be in [0,1] */
  public void throttle(double throttleValue) {
    throttleMultiplier = throttleValue;
  }

  public void offsetCurrentHeading(double offsetDegrees) {
    targetHeadingDegrees = getHeadingDegrees() + offsetDegrees;
  }

  public void setForwardTargetHeading() {
    targetHeadingDegrees = matchState.isBlue() ? 0 : 180;
  }

  public void stopDriving() {
    drive(0, 0, 0, true);
  }

  public Command stopDrivingCommand() {
    return new InstantCommand(this::stopDriving, this);
  }

  public double getDiffCurrentTargetYawDeg() {
    return Math.abs(getHeadingDegrees() - targetHeadingDegrees) % 360;
  }

  public boolean nearTarget() {
    return getDiffCurrentTargetYawDeg() < ShooterCal.ROBOT_HEADING_MARGIN_TO_SHOOT_DEGREES;
  }

  public boolean nearTargetAuto(){
    return getDiffCurrentTargetYawDeg() < 2.0;
  }

  /**
   * Provides no input to rotateOrKeepHeading for the input amount of time, allowing turning in
   * place towards the current target heading
   */
  public Command turnInPlace(double timeoutSec) {
    return new RunCommand(
            () -> {
              rotateOrKeepHeading(0, 0, 0, true, -1);
            })
        .withTimeout(timeoutSec);
  }

  /**
   * Calculate the amount the robot will overshoot in degrees, given rotational velocity in
   * degrees/second. Negative or positive values can be passed in, the function will adjust. It will
   * return the correct sign depending on our current velocity.
   */
  private double calculateYawOffsetDeg(double rotationalVelocityDeg) {
    double posRotationalVelocityDeg = Math.abs(rotationalVelocityDeg);
    double posOffsetDeg = yawOffsetMap.get(posRotationalVelocityDeg);
    return posOffsetDeg * Math.signum(lastSetChassisSpeeds.omegaRadiansPerSecond);
  }

  public void considerZeroingSwerveEncoders() {
    frontLeft.considerZeroingEncoder();
    frontRight.considerZeroingEncoder();
    rearLeft.considerZeroingEncoder();
    rearRight.considerZeroingEncoder();
  }

  public void throttleSpeed(boolean useHalfSpeed) {
    frontLeft.throttleSpeed(useHalfSpeed);
    frontRight.throttleSpeed(useHalfSpeed);
    rearLeft.throttleSpeed(useHalfSpeed);
    rearRight.throttleSpeed(useHalfSpeed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Throttle multiplier",
        () -> {
          return throttleMultiplier;
        },
        null);
    builder.addDoubleProperty(
        "Target Heading (deg)",
        () -> {
          return targetHeadingDegrees;
        },
        null);
    builder.addDoubleProperty("Gyro Yaw (deg)", () -> gyro.getYaw().getValueAsDouble(), null);
    builder.addDoubleProperty("Odometry X (m)", () -> getPose().getX(), null);
    builder.addDoubleProperty("Odometry Y (m)", () -> getPose().getY(), null);
    builder.addDoubleProperty(
        "Odometry Yaw (deg)", () -> getPose().getRotation().getDegrees(), null);
    builder.addDoubleProperty(
        "Front Left Abs Encoder (rad)", frontLeft::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Right Abs Encoder (rad)", frontRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Rear Left Abs Encoder (rad)", rearLeft::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Rear Right Abs Encoder (rad)", rearRight::getEncoderAbsPositionRad, null);
    builder.addDoubleProperty(
        "Front Left Module Pos (rad)", () -> frontLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (rad)", () -> frontRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Left Module Pos (rad)", () -> rearLeft.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear Right Module Pos (rad)", () -> rearRight.getPosition().angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Module Pos (deg)", () -> frontLeft.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Front Right Module Pos (deg)", () -> frontRight.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Rear Left Module Pos (deg)", () -> rearLeft.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Rear Right Module Pos (deg)", () -> rearRight.getPosition().angle.getDegrees(), null);
    builder.addDoubleProperty(
        "Front Left Distance (m)", () -> frontLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Front Right Distance (m)", () -> frontRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Left Distance (m)", () -> rearLeft.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Right Distance (m)", () -> rearRight.getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "Rear Right Velocity Error (mps)",
        () ->
            rearRight.desiredState.speedMetersPerSecond - rearRight.getState().speedMetersPerSecond,
        null);
    builder.addDoubleProperty("Keep Heading PID [0,1]", () -> KeepHeadingPID, null);
    builder.addDoubleProperty("Keep Heading FF [0,1]", () -> KeepHeadingFF, null);
    builder.addDoubleProperty("Rotation Controller Input", () -> rotControllerInput, null);
    builder.addDoubleProperty(
        "Yaw error",
        () -> (targetHeadingDegrees - getPose().getRotation().getDegrees()) % 360,
        null);
    builder.addDoubleProperty("Target Heading (tag detection)", () -> tagTargetHeading, null);
    builder.addDoubleProperty(
        "getDiffCurrentTargetYawDeg", () -> getDiffCurrentTargetYawDeg(), null);
    builder.addDoubleProperty(
        "Front left desired speed mps", () -> frontLeft.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front right desired speed mps", () -> frontRight.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear left desired speed mps", () -> rearLeft.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear right desired speed mps", () -> rearRight.desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front left current speed mps", () -> frontLeft.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front right current speed mps", () -> frontRight.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear left current speed mps", () -> rearLeft.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Rear right current speed mps", () -> rearRight.getState().speedMetersPerSecond, null);
    builder.addDoubleProperty(
        "Front left desired position", () -> frontLeft.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front right desired position", () -> frontRight.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear left desired position", () -> rearLeft.desiredState.angle.getRadians(), null);
    builder.addDoubleProperty(
        "Rear right desired position", () -> rearRight.desiredState.angle.getRadians(), null);
    builder.addBooleanProperty("Near Target heading", this::nearTarget, null);
    builder.addDoubleProperty(
        "yaw offset treemap value",
        () ->
            calculateYawOffsetDeg(
                Units.radiansToDegrees(lastSetChassisSpeeds.omegaRadiansPerSecond)),
        null);
  }
}
