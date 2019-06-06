package org.frc5687.deepspace.robot.utils;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.utils.math.RigidTransform2D;
import org.frc5687.deepspace.robot.utils.math.Rotation2D;
import org.frc5687.deepspace.robot.utils.math.Twist2D;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 */
public class Kinematics {
    private static final double Epsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist2D forwardKinematics(double leftWheelDelta, double rightWheelDelta) {
        double deltaVelocity = (rightWheelDelta - leftWheelDelta) / 2 * Constants.DriveTrain.WHEEL_SCRUB_FACTOR;
        double deltaRotation = deltaVelocity * 2 / Constants.DriveTrain.TRACK_WIDTH_INCHES;
        return forwardKinematics(leftWheelDelta, rightWheelDelta, deltaRotation);
    }

    /**
     * Forward kinematics using encoders and explicitly measured rotation (ex. from gyro)
     */
    public static Twist2D forwardKinematics(double leftWheelDelta, double rightWheelDelta, double deltaRotationRads) {
        final double dx = (leftWheelDelta + rightWheelDelta) / 2.0;
        return new Twist2D(dx, 0 , deltaRotationRads);
    }
    /**
     * For convenience, forward kinematic with an absolute rotation and previous rotation.
     */
    public static Twist2D forwardKinematics(Rotation2D previousHeading, double leftWheelDelta, double rightWheelDelta, Rotation2D currentHeading) {
        return forwardKinematics(leftWheelDelta, rightWheelDelta, previousHeading.inverse().rotateBy(currentHeading).getRadians());
    }

    public static RigidTransform2D integrateForwardKinematics(RigidTransform2D currentPose, double leftWheelDelta, double rightWheelDelta, Rotation2D currentHeading) {
        Twist2D withGyro = forwardKinematics(currentPose.getRotation(), leftWheelDelta, rightWheelDelta, currentHeading);
        return integrateForwardKinematics(currentPose, withGyro);
    }

    public static RigidTransform2D integrateForwardKinematics(RigidTransform2D  currentPose, Twist2D forwardKinematics) {
        return currentPose.transformBy(RigidTransform2D.exp(forwardKinematics));
    }

    public static class DriveVelocity {
        public final double left;
        public final double right;

        public DriveVelocity(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static DriveVelocity inverseKinematics(Twist2D velocity) {
        if (Math.abs(velocity._dTheta)  < Epsilon)  {
            return new DriveVelocity(velocity._dx,  velocity._dx);
        }
        double deltaV = Constants.DriveTrain.TRACK_WIDTH_INCHES * velocity._dTheta / (2 / Constants.DriveTrain.WHEEL_SCRUB_FACTOR);
        return new DriveVelocity(velocity._dx - deltaV, velocity._dx + deltaV);
    }
    


}
