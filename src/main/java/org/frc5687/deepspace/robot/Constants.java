package org.frc5687.deepspace.robot;

public class Constants {
    public static final int CYCLES_PER_SECOND = 50;

    public class DriveTrain {

        public static final double DEADBAND = 0.05;
        public static final double SPEED_SENSITIVITY = 0.75;
        public static final double ROTATION_SENSITIVITY = 0.75;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 0.75;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 0.75;
        public static final double LEFT_RATIO = 1.090909090909;
        public static final double RIGHT_RATIO = 1.090909090909;

        public static final boolean LEFT_MOTORS_INVERTED = true;
        public static final boolean RIGHT_MOTORS_INVERTED = false;
    }
    public class Gripper{
        public static final double VACUUM_SPEED = 0.2;
    }
    public static class Elevator {
        public static final double MAX_ELEVATOR_SPEED = 0.75;
        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY = 0.5;
        public static final boolean ELEVATOR_MOTOR_ELEVATOR = true;
    }
    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;
    }

    public class Roller {
        public static final double MAX_SPEED = 1.0;
        public static final boolean MOTOR_INVERTED = false;
        public static final double DEADBAND = 0.01;
        public static final double SENSITIVITY = 0.5;
    }

    public class Arm {
        public static final double MAX_INTAKE_SPEED = 0.3;
        public static final double SENSITIVITY = 0.5;
        public static final double DEADBAND = 0.1;
        public static final double MAX_DRIVE_SPEED = 0.5;
        public static final boolean MOTOR_INVERTED = true;
        public static final int SHOULDER_STALL_LIMIT = 10;
        public static final int SHOULDER_FREE_LIMIT = 80;
    }
    public class Auto {
        public static final double MIN_IMU_ANGLE = -180;
        public static final double MAX_IMU_ANGLE = 180;

        public static final double MAX_PITCH = 20.0;
        public static final double MAX_ROLL = 20.0;

        public class DriveToTarget{
            public static final double SPEED = 0.2;
            public static final double TURN_SPEED = 0.1;

            public static final double kPAngle = 0.04;
            public static final double kIAngle = 0.002;
            public static final double kDAngle = 0.2;

            public static final double kPDistance = 0.4;
            public static final double kIDistance = 0.000;
            public static final double kDDistance = 0.25;

            public static final double ANGLE_TOLERANCE = .5;
            public static final double DISTANCE_TOLERANCE = .5;

            public static final double TARGET_HEIGHT = 29;
            public static final double LIGHT_HEIGHT = 43;
        }
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

}