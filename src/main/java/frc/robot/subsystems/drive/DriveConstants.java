package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveConstants {
    /**
     * Invert the turning encoder, since the output shaft rotates in the opposite direction of the
     * steering motor in the MAXSwerve Module.
     */
    public static final boolean TURNING_ENCODER_INVERTED = true;

    /** Multiplier for wheel diameter based on empirical on-field measurement */
    // First number: adjustment for midwest
    // Second number: adjustment from midwest to practice field
    // Third number: adjustment from practice field to wisconsin
    // Fourth number: Wisconsin to Buckeye
    public static final double WHEEL_DIAMETER_FUDGE_FACTOR = 0.898 * 1.120 * 0.992 * 1.035;

    /** Calculations required for driving motor conversion factors and feed forward */
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = Constants.NEO_FREE_SPEED_RPM / 60,
        WHEEL_DIAMETER_METERS = Units.inchesToMeters(3) * WHEEL_DIAMETER_FUDGE_FACTOR,
        WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

    public static final double DRIVING_MOTOR_REDUCTION = 4.8;
    public static final double DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR = 1.0;
    public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND =
        DRIVE_WHEEL_FREE_SPEED_FUDGE_FACTOR
            * ((DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION);

    public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS =
        WHEEL_CIRCUMFERENCE_METERS / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND =
        DRIVING_ENCODER_POSITION_FACTOR_METERS / 60.0; // meters per second

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS =
        2 * Math.PI; // radians

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kCoast;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps

    public static final double TURN_MODULE_ENCODER_GEAR_RATIO = 1.0;
}
