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
        public static final double SWERVE_MODULE_RAMP_RATE = 0.0;
        public static final double SWERVE_MODULE_P         = 0.003;
        public static final double SWERVE_MODULE_I         = 0.00000155;
        public static final double SWERVE_MODULE_D         = 0;
    }

    /**
     * Encoders - Analog Ports and position offsets
     */
    public static class Encoder {
        /* The angle offsets tell us where zero is for each swerve module - Make all the gears face to the RIGHT */
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 355.576;
        public static final double FRONT_LEFT_ENCODER_OFFSET  = 357.539;
        public static final double REAR_LEFT_ENCODER_OFFSET   = 355.430;
        public static final double REAR_RIGHT_ENCODER_OFFSET  = 356.572;
    }
}