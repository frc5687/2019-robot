package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.RioLogger;

public class AutoDrive extends OutliersCommand {
    private double distance;
    private double speed;
    private PIDController distanceController;
    private PIDController angleController;
    private double _distancePIDOut;
    private double _anglePIDOut;
    private long endMillis;
    private long maxMillis;

    ;
    private boolean usePID;
    private boolean stopOnFinish;
    private double angle;
    private String debug;

    private DriveTrain driveTrain;
    private AHRS imu;

    private double kPdistance = 0.15; // .05;
    private double kIdistance = 0.000; // .001;
    private double kDdistance = 0.3; //.1;
    private double kTdistance = 0.5;

    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private double kTangle;


    public AutoDrive(DriveTrain driveTrain, AHRS imu, double distance, double speed, String debug) {
        this(driveTrain, imu, distance, speed, true, true, 0, debug);
    }

    public AutoDrive(DriveTrain driveTrain, AHRS imu, double distance, double speed, long maxMillis, String debug) {
        this(driveTrain, imu, distance, speed, false, true, 1000, maxMillis, debug);
    }

    public AutoDrive(DriveTrain driveTrain, AHRS imu, double distance, double speed, boolean usePID, boolean stopOnFinish, long maxMillis, String debug) {
        this(driveTrain, imu, distance, speed, usePID, stopOnFinish, 1000, maxMillis, debug);
    }


    /***
     * Drives for a set distance at a set speed.
     * @param distance Distance to drive
     * @param speed Speed to drive
     * @param usePID Whether to use pid or not
     * @param stopOnFinish Whether to stop the motors when we are done
     * @param angle The angle to drive, in degrees.  Pass 1000 to maintain robot's hading.
     * @param maxMillis Maximum time in millis to allow the command to run
     */
    public AutoDrive(DriveTrain driveTrain, AHRS imu, double distance, double speed, boolean usePID, boolean stopOnFinish, double angle, long maxMillis, String debug) {
        requires(driveTrain);
        this.speed = speed;
        this.distance = distance;
        this.usePID = usePID;
        this.stopOnFinish = stopOnFinish;
        this.angle = angle;
        this.maxMillis = maxMillis;
        this.debug = debug;
        this.driveTrain = driveTrain;
        this.imu = imu;
    }

    @Override
    protected void initialize() {
        driveTrain.resetDriveEncoders();
        this.endMillis = maxMillis == 0 ? Long.MAX_VALUE : System.currentTimeMillis() + maxMillis;
        driveTrain.enableBrakeMode();
        if (usePID) {
            metric("kP", kPdistance);
            metric("kI", kIdistance);
            metric("kD", kDdistance);
            metric("kT", kTdistance);

            distanceController = new PIDController(kPdistance, kIdistance, kDdistance, speed, driveTrain, new DistanceListener(), 0.01);
            distanceController.setAbsoluteTolerance(kTdistance);
            distanceController.setOutputRange(-speed, speed);
            distanceController.setSetpoint(driveTrain.getDistance() + distance);
            distanceController.enable();
        }

        angleController = new PIDController(kPangle, kIangle, kDangle, imu, new AngleListener(), 0.01);
        angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        double maxSpeed = speed * Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE;
        metric("angleMaxSpeed", maxSpeed);
        metric("setPoint", driveTrain.getYaw());
        angleController.setOutputRange(-maxSpeed, maxSpeed);
        angleController.setContinuous();

        // If an angle is supplied, use that as our setpoint.  Otherwise get the current heading and stick to it!
        angleController.setSetpoint(angle==1000?driveTrain.getYaw():angle);
        angleController.enable();

        info("Auto Drive initialized: " + (debug==null?"":debug));
    }

    @Override
    protected void execute() {

        driveTrain.setPower(_distancePIDOut + _anglePIDOut , _distancePIDOut - _anglePIDOut, true); // positive output is clockwise
        metric("onTarget", distanceController == null ? false : distanceController.onTarget());
        metric("imu", driveTrain.getYaw());
        metric("distance", driveTrain.pidGet());
        metric("turnPID", _anglePIDOut);
        metric("distancePID", _distancePIDOut);

    }

    @Override
    protected boolean isFinished() {
        if (maxMillis>0 && endMillis!=Long.MAX_VALUE && System.currentTimeMillis() > endMillis) {
            info("AutoDrive for " + maxMillis + " timed out.");
            DriverStation.reportError("AutoDrive for " + maxMillis + " timed out.", false);
            return true; }
        if (usePID) {
            if (distanceController.onTarget()) {
               DriverStation.reportError("AutoDrive stoped at " + driveTrain.getDistance(),false);

            }
        } else {
            info("AutoDrive nopid complete at " + driveTrain.getDistance() + " inches");
            return distance == 0 ? true : distance < 0 ? (driveTrain.getDistance() < distance) : (driveTrain.getDistance() >  distance);
        }
        return false;
    }



    @Override
    protected void end() {
        info("AutoDrive Finished (" + driveTrain.getDistance() + ", " + (driveTrain.getYaw() - angleController.getSetpoint()) + ") " + (debug==null?"":debug));
        DriverStation.reportError("AutoDrive Finished (" + driveTrain.getDistance() + ", " + (driveTrain.getYaw() - angleController.getSetpoint()) + ") " + (debug==null?"":debug), false);
        driveTrain.enableCoastMode();
        angleController.disable();
        if (distanceController!=null) {
            distanceController.disable();
        }
        if (stopOnFinish) {
            info("Stopping at ." + driveTrain.getDistance());
            driveTrain.enableBrakeMode();
            driveTrain.setPower(0, 0, true);
        }
    }


    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }
    }

    private class DistanceListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _distancePIDOut = output;
            }
        }
    }


}
