package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final double INTAKE_POSITION_WHEN_HORIZONTAL_DEGREES = 70.0;

  public static final double INPUT_ABS_ENCODER_GEAR_RATIO = 3.0;
  public static final double INPUT_ABS_ENCODER_WRAP_INCREMENT_DEGREES =
      360.0 / INPUT_ABS_ENCODER_GEAR_RATIO;
  public static final double INTAKING_TIMEOUT_SEC = 7.5; // TODO check this

  public static final double PIVOT_MOTOR_GEAR_RATIO = 97.06;
}
