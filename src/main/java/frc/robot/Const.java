package frc.robot;

/**
 * The constants class stores all of our robot wiring constants.
 */
public class Const {
    public static class CAN {
        public static final int FRONT_RIGHT_DRIVE_MOTOR =  4; /* Module 1 */
        public static final int FRONT_LEFT_DRIVE_MOTOR  =  1; /* Module 2 */
        public static final int REAR_LEFT_DRIVE_MOTOR   =  10; /* Module 3 */
        public static final int REAR_RIGHT_DRIVE_MOTOR  =  7; /* Module 4 */

        public static final int FRONT_RIGHT_PIVOT_MOTOR =  6; /* Module 1 */
        public static final int FRONT_LEFT_PIVOT_MOTOR  =  3; /* Module 2 */
        public static final int REAR_LEFT_PIVOT_MOTOR   =  12; /* Module 3 */
        public static final int REAR_RIGHT_PIVOT_MOTOR  =  9; /* Module 4 */

        public static final int FRONT_RIGHT_ENCODER     = 5; /* Module 1 */
        public static final int FRONT_LEFT_ENCODER      = 2; /* Module 2 */
        public static final int REAR_LEFT_ENCODER       = 11; /* Module 3 */
        public static final int REAR_RIGHT_ENCODER      = 8; /* Module 4 */

        public static final int POWER_DISTRIBUTION_PANEL = 59;
    }

    public static class PID {

        public static final int ANGLE_STANDING_SLOT = 0;
        public static final int ANGLE_MOVING_SLOT = 1;

        public static final double SWERVE_DRIVE_ANGLE_STANDING_P = 0.0055;
        public static final double SWERVE_DRIVE_ANGLE_STANDING_I = 0;
        public static final double SWERVE_DRIVE_ANGLE_STANDING_D = 0;

        public static final double SWERVE_DRIVE_ANGLE_MOVING_P = 0.004;
        public static final double SWERVE_DRIVE_ANGLE_MOVING_I = 0.0000001;
        public static final double SWERVE_DRIVE_ANGLE_MOVING_D = 0;

        public static final double SWERVE_DRIVE_ACCEPTABLE_ERROR = 0.25;

        public static final double SWERVE_MODULE_RAMP_RATE = 0.02;
        
        public static final double SWERVE_MODULE_P = 0.003;
        public static final double SWERVE_MODULE_I = 0.00000155;
        public static final double SWERVE_MODULE_D = 0;
    }

    /**
     * Encoders - Analog Ports and position offsets
     */
    public static class Angle {
        /* The angle offsets tell us where zero is for each swerve module - Make all the gears face to the RIGHT */
        public static final double FRONT_RIGHT_OFFSET = 353.584;
        public static final double FRONT_LEFT_OFFSET  = 156.230;
        public static final double REAR_LEFT_OFFSET   = 82.354;
        public static final double REAR_RIGHT_OFFSET  = 78.223;
    }
}