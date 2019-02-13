package org.frc5687.deepspace.robot;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and then in numerical order.
     * Note that for CAN, ids must be unique per device type, but not across types.
     * Thus, you cannot have two SparkMax controllers with Id 0, but you can have a SparkMax with Id 0 and a TalonSRX with Id 0.
      */
    public static class CAN {

        /*  Example:
                        public static final int LEFT_MASTER_SPARK= 1;
                        */
        public static class SPARKMAX {
            public static final int DRIVE_LEFT_MASTER = 11;
            public static final int DRIVE_RIGHT_MASTER = 6;
            public static final int DRIVE_LEFT_FOLLOWER = 14;
            public static final int DRIVE_RIGHT_FOLLOWER = 8;
            public static final int ELEVATOR_MOTOR = 5;
            public static final int ROLLER = 9;
            public static final int ARM = 16;
            public static final int INTAKE_VACUUM = 0; //vacuum motor for intake is currently not set up, set to 0 for now.
            public static final int STILT = 13;
        }
        public static class TALONSRX{
            public static final int GRIPPER_VACUUM = 0;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order.
     * Note that for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {
        /*  Example:
        public static final int ARM_VICTORSP = 0;
        */
    }

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order.
     * Note that for PCM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PCM {
        /* Example:
        public static final int LEFT_PINCER_OPEN = 5;
        */

        //PCM ports are not in the robot yet, using 0 and 1.
        public static final int SPEAR_OPEN = 2;
        public static final int SPEAR_CLOSE = 3;
        public static final int WRIST_DOWN = 1;
        public static final int WRIST_UP = 0;
        public static final int SHIFTER_HIGH = 4;
        public static final int SHIFTER_LOW = 5;
    }

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical order.
     * Note that only on device can be connected to each breaker, so the numbers should be unique.
     */
    public static class PDP {
        /* Example:
        public static final int ARM_VICTORSP = 0;
        */
        public static final int GRIPPER_VACCUUM = 8;
    }

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical order.
     * Note that for Analog only one device can connect to each port, so the numbers should be unique.
     */
    public static class Analog {
        public static final int BALL_IR = 1;
        /*
        public static final int ARM_POTENTIOMETER = 7;
         */
        public static final int FRONT_IR = 0;
    }

    /**
     * There should be an entry here for each DIO port, preferrably in numerical order.
     * Note that for DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int ELEVATOR_B = 0;
        public static final int ELEVATOR_A = 1;
        public static final int ELEVATOR_TOP_HALL = 2;
        public static final int ELEVATOR_BOTTOM_HALL = 3;

        public static final int ARM_LOW_HALL = 9;
        public static final int ARM_INTAKE_HALL = 8;
        public static final int ARM_SECURE_HALL = 6;
        public static final int ARM_STOWED_HALL = 7;

        public static final int DRIVE_LEFT_A = 23;
        public static final int DRIVE_LEFT_B = 21;
        public static final int DRIVE_RIGHT_A = 19;
        public static final int DRIVE_RIGHT_B = 13;


        /* Example:
        public static final int ARM_FRONT_LIMIT = 0;
        */
    }

}
