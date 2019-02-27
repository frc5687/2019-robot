package org.frc5687.deepspace.robot;

public class Constants {
    /**
     *
     */
    public static final int CYCLES_PER_SECOND = 50;
    public static final int TICKS_PER_UPDATE = 10;

    public class DriveTrain {

        public static final double DEADBAND = 0.1;
        public static final double SPEED_SENSITIVITY = 0.80;
        public static final double ROTATION_SENSITIVITY = 0.75;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 0.9;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 0.9;
        public static final double TURNING_SENSITIVITY_HIGH_GEAR = 0.9;
        public static final double TURNING_SENSITIVITY_LOW_GEAR = 0.9;

        public static final double LEFT_RATIO = 1.090909090909;
        public static final double RIGHT_RATIO = 1.090909090909;

        public static final boolean LEFT_MOTORS_INVERTED = true;
        public static final boolean RIGHT_MOTORS_INVERTED = false;
    }
    public class Intake {
        //Roller
        public static final double HIGH_POW = 1.0;
        public static final double LOW_POW = -HIGH_POW;
        public static final double ROLLER_SPEED = 0.5;
        public static final double MAX_ROLLER_SPEED = 1.0;
        public static final boolean MOTOR_INVERTED = false;
        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY = 0.5;
        public static final long ROLLER_TIME_MILLI_SEC = 1000;
        public static final int CARGO_DETECTED_THRESHOLD = 600;
        //Wrist
        public static final long RAISE_WRIST_MILLI_SEC = 500;
        public static final long LOWER_WRIST_MILLI_SEC = 500;
        //Claw
        public static final long OPEN_CLAW_MILLI_SEC = 40;
        public static final long CLOSE_CLAW_MILLI_SEC = 40;
        public static final long CLAW_RAISE_WRIST_MILLI_SEC = 300;
        public static final long CLAW_LOWER_WRIST_MILLI_SEC = 300;


        public static final long CARGO_EJECT_MILLIS = 200;
        public static final double CARGO_EJECT_SPEED = -0.5;

    }
    public class Shifter {
        public static final long STOP_MOTOR_TIME = 60;
        public static final long SHIFT_TIME = 60;

        public static final double SHIFT_UP_THRESHOLD = 50; // in inches per second graTODO tune
        public static final double SHIFT_DOWN_THRESHOLD = 40; // in inches per second TODO tune

        public static final long AUTO_WAIT_PERIOD = 500;
        public static final long MANUAL_WAIT_PERIOD = 3000;

    }

    public static class Elevator {
        public static final double MAX_SPEED = 1.0;
        public static final double MAX_SPEED_UP = 1.0;
        public static final double MAX_SPEED_DOWN = 0.8;
        public static final double SPEED_UP = 1.0;
        public static final double SPEED_DOWN = 0.8;

        public static final double JELLO_SPEED_UP = 0.2;
        public static final double JELLO_SPEED_DOWN = 0.2;

        public static final double TOP_JELLO_ZONE = 500;
        public static final double BOTTOM_JELLO_ZONE = 1000;

        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY = 0.5;
        public static final boolean ELEVATOR_MOTOR_INVERTED = true;
        public static final int TOLERANCE = 10;
        public static final double MAX_VELOCITY_IPS = 27.0;
        public static final double TICKS_PER_INCH = 111.1111111;
        public static final double STEPS_UP = 10;
        public static final double STEPS_DOWN = 30;
        public static final double TICKS_PER_STEP = 50;
        public static final double MIN_SPEED = 0.2;
        public static final double GOAL_SPEED = 0.5;

        public static class PID {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
        public static class Path {
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }
    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;

    }

    public class Arm {
        public static final double SENSITIVITY = 0.5;
        public static final double DEADBAND = 0.4;

        public static final double MAX_DRIVE_SPEED = 1;
        public static final double HOLD_SPEED = 0.01;
        public static final double STOW_SPEED = -0.2;

        public static final boolean LEFT_MOTOR_INVERTED = true;
        public static final boolean RIGHT_MOTOR_INVERTED = false;

        public static final int SHOULDER_STALL_LIMIT = 10;
        public static final int SHOULDER_FREE_LIMIT = 80;

        public static final double kI = 0;
        public static final double kP = 0.1;
        public static final double kD = 0;
        public static final double TOLERANCE = 2.0;
        public static final double SPEED = 1;
        public static final double SPEED_UP = 1;
        public static final double SPEED_DOWN = 1;

        public static final double DEGREES_PER_TICK = 90.0 / 70.0;

        public static final double STOWED_ANGLE = 0.0;
    }

    public class Lights {
        public static final double SOLID_BLUE = 0.87;
        public static final double PULSING_BLUE = -0.09;
        public static final double BEATING_BLUE = 0.23;

        public static final double SOLID_RED = 0.61;
        public static final double PULSING_RED = -0.11;
        public static final double BEATING_RED = 0.25;

        public static final double SOLID_GREEN = 0.77;
        public static final double PULSING_GREEN = 0.77; // replace
        public static final double BEATING_GREEN = 0.00; // unused

        public static final double SOLID_PURPLE = 0.91;
        public static final double PULSING_PURPLE = 0.05;
        public static final double BEATING_PURPLE = 0.00;

        public static final double SOLID_ORANGE = 0.06;
        public static final double PULSING_ORANGE = 0.07;
        public static final double BEATING_ORANGE = 0.08;

        public static final double SOLID_YELLOW = 0.69;
        public static final double PULSING_YELLOW = 0.10;
        public static final double BEATING_YELLOW = 0.11;

        public static final double SOLID_BLACK = 0.99;

        public static final double SOLID_HOT_PINK = 0.57;

        public static final double CONFETTI = -0.87;
    }
    public class Stilt {
        public static final boolean MOTOR_INVERTED = false;
        public static final double SENSITVITY = 0.1;
        public static final double DEADBAND = 0.1;
        public static final double MAX_UP_SPEED= 0.5;
        public static final double MAX_DOWN_SPEED = 0.4;
        public static final double STILT_HOLD_SPEED = 0.07;
    }


    /*
     There should be a nested static class for each subsystem and for each autonomous command that needs tuning constants.
     For example:
    public static class DriveTrain {
        public static final double DEADBAND = 0.3;
        public static final double SENSITIVITY_LOW_GEAR = 0.8;
        public static final double SENSITIVITY_HIGH_GEAR = 1.0;
        public static final double ROTATION_SENSITIVITY = 1.0;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 1.0;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 0.8;
    }
     */


    public static class Auto {
        public static final double MIN_IMU_ANGLE = -180;
        public static final double MAX_IMU_ANGLE = 180;

        public static final double MAX_PITCH = 20.0;
        public static final double MAX_ROLL = 20.0;
        public static final double IR_THRESHOLD = 24.0;
        public static class Climb {
            public static final double STILT_SPEED = 0.55;
            public static final double STILT_HOLD_SPEED = 0.4;
            public static final double RAISE_STILT_SPEED = -0.2;

            public static final double ARM_SPEED = 0.4;
            public static final double ARM_SLOW_SPEED = 0.2;
            public static final double ARM_HOLD_SPEED = 0.0;
            public static final double RAISE_ARM_SPEED = -0.2;

            public static final double CONTACT_ANGLE = 95.0;
            public static final double SLOW_ANGLE = 165.0;
            public static final double BOTTOM_ANGLE = 175.0;

            public static final double ARM_RETRACT_ANGLE = 165.0;

            public static final double INITIAL_ARM_SPEED = 0.6;

            public static final double WHEELIE_FORWARD_SPEED = 1.0;

            public static final double DRIVE_FORWARD_SPEED = 0.2;

            public static final double PARK_SPEED = 0.01;
            public static final double PARK_DISTANCE = 18;

            public static final long STILT_TIMEOUT = 2000;

        }
        public static class AutoAlignWhileDriving {

            public static final double SPEED = 1.0;

            public static final double kP = 0.06;
            public static final double kI = 0.000;
            public static final double kD = 0.1;
            public static final double MINIMUM_SPEED = 0.2;
            public static final double STEADY_TIME = 60;
            public static final double TURN_SPEED = 0.2;
            public static final double TOLERANCE = .5;
            public static final double DEADBAND = .3;
            public static final double THRESHOLD = 0.5;
        }
    }
}