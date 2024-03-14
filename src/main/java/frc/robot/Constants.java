// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  /*
   * 0 - whatever pipleine is on
   * 1 - off
   * 2 - blinking
   * 3 - on
   */
  public static enum limelightLedMode {
    PIPELINE_MODE,
    OFF,
    BLINK,
    ON
  }

  /*
   * 0 - limelight
   * 1 - driver
   */
  public static enum limelightCamMode {
    VISION_PROCESSING,
    DRIVER_CAMERA
  }

  public static enum limelightPipeline {
    /** Pipeline 0 in LL */
    TAG_PIPELINE,
    /** Pipeline 1 in LL */
    NOTE_PIPELINE,
    PIPELINE2,
    PIPELINE3,
    PIPELINE4,
    PIPELINE5,
    PIPELINE6,
    PIPELINE7,
    PIPELINE8,
    PIPELINE9
  }

  public static final double NEO_FREE_SPEED_RPM = 5676.0;
  public static final double KRAKEN_FREE_SPEED_RPM = 6000.0;

  public static final double PLACEHOLDER_DOUBLE = 0.0;
  public static final int PLACEHOLDER_INT = 0;
  public static final float PLACEHOLDER_FLOAT = 0;
  public static final Transform2d PLACEHOLDER_TRANSFORM2D = new Transform2d();

  public static final int SPARK_INIT_RETRY_ATTEMPTS = 5;

  /* time between compute cycles */
  public static final double PERIOD_TIME_SECONDS = 0.02;

  public static final double BROWNOUT_VOLTAGE = 5.0;

  public static final double NOTE_HEIGHT_INCHES = 2.0;

  public static final double ODOMETRY_ERROR = 0.2; // arbitrary value for now
}
