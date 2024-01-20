package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class IntakeCal {

        /**
         * PID values for the intake pivot motor
         * input degrees, output volts
         */
        public static final double INTAKE_PIVOT_P = Constants.PLACEHOLDER_DOUBLE,
                        INTAKE_PIVOT_I = Constants.PLACEHOLDER_DOUBLE, INTAKE_PIVOT_D = Constants.PLACEHOLDER_DOUBLE;

        /** max velocity and acceleration (deg per second) for intake pivot motor */
        public static final double PIVOT_MAX_VELOCITY_DEG_PER_SECOND = Constants.PLACEHOLDER_DOUBLE,
                        PIVOT_MAX_ACCELERATION_DEG_PER_SECOND_SQUARED = Constants.PLACEHOLDER_DOUBLE;

        /**
         * intake positions (degrees) where 90 degrees represents the intake at the
         * shooter and 0 degrees is down
         */
        public static final double INTAKE_DEPLOYED_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        INTAKE_STOWED_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        INTAKE_SAFE_POSITION_DEGREES = Constants.PLACEHOLDER_DOUBLE;

        public static final double INTAKE_MARGIN_DEGREES = Constants.PLACEHOLDER_DOUBLE,
                        CONVEYOR_ZONE_THRESHOLD_DEGREES = Constants.PLACEHOLDER_DOUBLE;

        /** intake power [-1.0,1.0] */
        public static final double INTAKING_POWER = 1.0,
                        REVERSE_INTAKING_POWER = -1.0;

        /* input degrees, output volts */
        public static final SimpleMotorFeedforward INTAKE_PIVOT_FEEDFORWARD = new SimpleMotorFeedforward(
                        Constants.PLACEHOLDER_DOUBLE, Constants.PLACEHOLDER_DOUBLE);
        public static final double ARBITRARY_INTAKE_PIVOT_FEEDFORWARD_VOLTS = Constants.PLACEHOLDER_DOUBLE;

        /** start and offset degrees for intake absolute encoder */
        public static final double INTAKE_ABSOLUTE_ENCODER_ZERO_OFFSET_DEG = Constants.PLACEHOLDER_DOUBLE,
                        INTAKE_ABSOLUTE_ENCODER_START_POS_DEG = Constants.PLACEHOLDER_DOUBLE;

        public static final int PIVOT_MOTOR_CURRENT_LIMIT_AMPS = Constants.PLACEHOLDER_INT;
        public static final int PIVOT_SPARK_INIT_RETRY_ATTEMPTS = Constants.PLACEHOLDER_INT;

}
