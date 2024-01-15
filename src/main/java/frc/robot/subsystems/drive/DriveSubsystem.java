package frc.robot.subsystems.drive;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lights;
import frc.robot.utils.GeometryUtils;

public class DriveSubsystem extends SubsystemBase {
    private double targetHeadingDegrees;
    private Lights lights;

    // Create SwerveModules
    public final SwerveModule frontLeft = new SwerveModule(
            RobotMap.FRONT_LEFT_DRIVING_CAN_ID,
            RobotMap.FRONT_LEFT_TURNING_CAN_ID,
            DriveCal.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

    public final SwerveModule frontRight = new SwerveModule(
            RobotMap.FRONT_RIGHT_DRIVING_CAN_ID,
            RobotMap.FRONT_RIGHT_TURNING_CAN_ID,
            DriveCal.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

    public final SwerveModule rearLeft = new SwerveModule(
            RobotMap.REAR_LEFT_DRIVING_CAN_ID,
            RobotMap.REAR_LEFT_TURNING_CAN_ID,
            DriveCal.BACK_LEFT_CHASSIS_ANGULAR_OFFSET_RAD);

    public final SwerveModule rearRight = new SwerveModule(
            RobotMap.REAR_RIGHT_DRIVING_CAN_ID,
            RobotMap.REAR_RIGHT_TURNING_CAN_ID,
            DriveCal.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD);

    private final Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_CAN_ID);
    private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    public Optional<Pose2d> targetPose = Optional.empty();
    /** include path planner initialization here */
    private MedianFilter pitchFilter = new MedianFilter(Constants.PLACEHOLDER_INT);
    private double latestFilteredPitchDeg = Constants.PLACEHOLDER_DOUBLE;

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveDrive.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(Constants.PLACEHOLDER_DOUBLE), getModulePositions());

    /** Multiplier for drive speed, does not affect trajectory following */
    private double throttleMultiplier = 1.0;
    private BooleanSupplier isTimedMatch;

    public DriveSubsystem(Lights lightsSubsystem, BooleanSupplier isTimedMatchFunc) {
        /** Need to factory default settings for gyro, but no function exists */
        gyro.reset();
        /** Need to configure a mount pose for gyro, but no function exist */
        this.lights = lightsSubsystem;
        this.isTimedMatch = isTimedMatchFunc;
    }

    public double getFilteredPitch() {
        return latestFilteredPitchDeg - Constants.SwerveSubsystem.IMU_PITCH_BIAS_DEG;
    }

    @Override
    public void periodic() {
        latestFilteredPitchDeg = pitchFilter.calculate(gyro.getPitch().getValue());

        // Update the odometry in the periodic block
        frontLeft.periodic();
        frontRight.periodic();
        rearLeft.periodic();
        rearRight.periodic();
        odometry.update(Rotation2d.fromDegrees(gyro.getYaw().getValue()), getModulePositions());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
        };
    };

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

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // Just update the translation, not the yaw
        Pose2d resetPose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(gyro.getYaw().getValue()));
        odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw().getValue()), getModulePositions(), resetPose);
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
        resetYawToAngle(0.0);
    }

    /**
     * Correction for swerve second order dynamics issue. Borrowed from 254:
     * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
     * Discussion:
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose = new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
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
        frontLeft.setDesiredState(new SwerveModuleState(0, frontLeftCurrRot));
        frontRight.setDesiredState(new SwerveModuleState(0, frontRightCurrRot));
        rearLeft.setDesiredState(new SwerveModuleState(0, rearLeftCurrRot));
        rearRight.setDesiredState(new SwerveModuleState(0, rearRightCurrRot));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Desired speed of the robot in the x direction (forward),
     *                      [-1,1].
     * @param ySpeed        Desired speed of the robot in the y direction
     *                      (sideways), [-1,1].
     * @param rot           Desired angular rate of the robot, [-1,1].
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (isTimedMatch.getAsBoolean()
                && DriverStation.isTeleop()
                && DriverStation.getMatchTime() < 0.3) {
            setX();
            return;
        }
        if (xSpeed == 0 && ySpeed == 0 && rot == 0) {
            setNoMove();
            return;
        }
        xSpeed *= Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
        ySpeed *= Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND;
        rot *= Constants.SwerveDrive.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

        xSpeed *= throttleMultiplier;
        ySpeed *= throttleMultiplier;
        rot *= throttleMultiplier;

        ChassisSpeeds desiredChassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getYaw().getValue()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        lastSetChassisSpeeds = desiredChassisSpeeds;

        var swerveModuleStates = Constants.SwerveDrive.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        targetHeadingDegrees = getHeadingDegrees();
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Constants.SwerveDrive.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
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
        return gyro.getRate() * (Constants.SwerveDrive.GYRO_REVERSED ? Constants.PLACEHOLDER_DOUBLE : Constants.PLACEHOLDER_DOUBLE);
    }
    

}