package frc.robot;

/**
 * The constants class stores all of our robot wiring constants.
 */
public class Const {
    public static class CAN {
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 5; /* Module 1 */
        public static final int FRONT_LEFT_DRIVE_MOTOR  = 6; /* Module 2 */
        public static final int REAR_LEFT_DRIVE_MOTOR   = 7; /* Module 3 */
        public static final int REAR_RIGHT_DRIVE_MOTOR  = 8; /* Module 4 */

        public static final int FRONT_RIGHT_PIVOT_MOTOR = 1; /* Module 1 */
        public static final int FRONT_LEFT_PIVOT_MOTOR  = 2; /* Module 2 */
        public static final int REAR_LEFT_PIVOT_MOTOR   = 3; /* Module 3 */
        public static final int REAR_RIGHT_PIVOT_MOTOR  = 4; /* Module 4 */

        public static final int POWER_DISTRIBUTION_PANEL = 59;
    }

    public static class PID {
        public static final double SWERVE_MODULE_RAMP_RATE = 0;
        public static final double SWERVE_MODULE_P         = 0.003;
        public static final double SWERVE_MODULE_I         = 0.00000155;
        public static final double SWERVE_MODULE_D         = 0;
    }

    /**
     * Encoders - Analog Ports and position offsets
     */
    public static class Encoder {
        public static final int FRONT_RIGHT_ANALOG_ENCODER = 0; /* Module 1 */
        public static final int FRONT_LEFT_ANALOG_ENCODER  = 1; /* Module 2 */
        public static final int REAR_LEFT_ANALOG_ENCODER   = 2; /* Module 3 */
        public static final int REAR_RIGHT_ANALOG_ENCODER  = 3; /* Module 4 */

        /* The encoder offsets tell us where zero is for each motor. */
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 146.27;
        public static final double FRONT_LEFT_ENCODER_OFFSET  = 134.60;
        public static final double REAR_LEFT_ENCODER_OFFSET   = 59.34;
        public static final double REAR_RIGHT_ENCODER_OFFSET  = 267.2;
    }
}