// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double NEO_FREE_SPEED_RPM = 5676.0;

  public static final double PLACEHOLDER_DOUBLE = 0.0;
  public static final int PLACEHOLDER_INT = 0;

  public final class SwerveDrive {
    /** Distance between centers of right and left wheels on robot */
    public static final double TRACK_WIDTH_METERS = PLACEHOLDER_DOUBLE;

    /**
     * Driving Parameters - Note that these are not the maximum capable speeds of
     * the robot, rather
     * the allowed maximum speeds
     */
    public static final double MAX_SPEED_METERS_PER_SECOND = PLACEHOLDER_DOUBLE,
        MAX_ANGULAR_SPEED_RAD_PER_SECONDS = Math.PI * 2;

    /** Distance between front and back wheels on robot */
    public static final double WHEEL_BASE_METERS = PLACEHOLDER_DOUBLE;
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
        new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

    public static final boolean GYRO_REVERSED = false;
  }

  public final class SwerveSubsystem {
    public static final double IMU_PITCH_BIAS_DEG = PLACEHOLDER_DOUBLE;
  }
}
