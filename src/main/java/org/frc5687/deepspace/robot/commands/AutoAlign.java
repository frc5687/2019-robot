package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.RioLogger;

public class AutoAlign extends OutliersCommand implements PIDOutput {
    private PIDController controller;
    private double endTime;
    private double angle;
    private double speed;
    private long _timeout = 2000;

    private double pidOut;

    private long _onTargetSince;
    private long startTimeMillis;
    private long _endTimeMillis;
    private boolean _aborted = false;

    private DriveTrain driveTrain;
    private AHRS imu;

    private String _message = "";

    private DriveTrainBehavior _driveTrainBehavior = DriveTrainBehavior.bothSides;

    private double _tolerance;
    public AutoAlign(Robot robot, double angle) {
        this(robot, angle, Constants.Auto.Align.SPEED);
    }

    public AutoAlign(Robot robot, double angle, double speed) {
        this(robot.getDriveTrain(), robot.getIMU(), angle, speed);
    }

    public AutoAlign(DriveTrain driveTrain, AHRS imu, double angle, double speed) {
        this(driveTrain, imu, angle, speed, 2000);
    }

    public AutoAlign(DriveTrain driveTrain, AHRS imu, double angle, double speed, long timeout) {
        this(driveTrain, imu, angle, speed, timeout, Constants.Auto.Align.TOLERANCE, "");
    }

    public AutoAlign(Robot robot, double angle, long timeout, double tolerance) {
        this(robot.getDriveTrain(), robot.getIMU(), angle, Constants.Auto.Align.SPEED, timeout, tolerance, "");
    }

    public AutoAlign(Robot robot, double angle, double speed, long timeout, double tolerance) {
        this(robot.getDriveTrain(), robot.getIMU(), angle, speed, timeout, tolerance, "");
    }

    public AutoAlign(DriveTrain driveTrain, AHRS imu, double angle, double speed, long timeout, double tolerance, String message) {
        this(driveTrain, imu, angle, speed, timeout, tolerance, DriveTrainBehavior.bothSides, message);
    }

    public AutoAlign(Robot robot, double angle, double speed, long timeout, double tolerance, DriveTrainBehavior driveTrainBehavior, String message) {
        this(robot.getDriveTrain(), robot.getIMU(), angle, speed, timeout, tolerance, driveTrainBehavior, message);
    }

    public AutoAlign(DriveTrain driveTrain, AHRS imu, double angle, double speed, long timeout, double tolerance, DriveTrainBehavior driveTrainBehavior, String message) {
        requires(driveTrain);
        this.angle = angle;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.imu = imu;
        _timeout = timeout;
        _tolerance = tolerance;
        _driveTrainBehavior = driveTrainBehavior;
        _message = message;
    }

    @Override
    protected void initialize() {
        double kP = Constants.Auto.Align.kP; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kI = Constants.Auto.Align.kI; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kD = Constants.Auto.Align.kD; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        controller = new PIDController(kP, kI, kD, imu, this, 0.01);
        controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        controller.setOutputRange(-speed, speed);
        controller.setAbsoluteTolerance(_tolerance);
        controller.setContinuous();
        controller.setSetpoint(angle);
        controller.enable();
        info(_message + " initialized to " + angle + " at " + speed);
        info("kP="+kP+" , kI="+kI+", kD="+kD + ",T="+ Constants.Auto.Align.TOLERANCE);
        startTimeMillis = System.currentTimeMillis();
        _endTimeMillis = startTimeMillis + _timeout;
    }

    @Override
    protected void execute() {
        //actOnPidOut();
        // Check pitch and tilt
        double pitch = imu.getPitch();
        double roll = imu.getRoll();

        if (Math.abs(pitch) > Constants.Auto.MAX_PITCH) {
            error("Excessive pitch detected (" + pitch + ")");
            this.controller.disable();
            _aborted = true;
        }

        if (Math.abs(roll) > Constants.Auto.MAX_ROLL) {
            error("Excessive roll detected (" + roll + ")");
            this.controller.disable();
            _aborted = true;
        }


        metric("onTarget", controller.onTarget());
        metric("imu", imu.getYaw());
        //SmartDashboard.putData("AutoAlign/pid", controller);
    }

    private void actOnPidOut() {
        if (pidOut > 0 && pidOut < Constants.Auto.Align.MINIMUM_SPEED) {
            pidOut = Constants.Auto.Align.MINIMUM_SPEED;
        }
        if (pidOut < 0 && pidOut > -Constants.Auto.Align.MINIMUM_SPEED) {
            pidOut = -Constants.Auto.Align.MINIMUM_SPEED;
        }
        if (_driveTrainBehavior == DriveTrainBehavior.bothSides) {
            driveTrain.setPower(pidOut, -pidOut, true); // positive output is clockwise
        } else if (_driveTrainBehavior == DriveTrainBehavior.rightOnly) {
            driveTrain.setPower(0, -pidOut);
        } else if (_driveTrainBehavior == DriveTrainBehavior.leftOnly) {
            driveTrain.setPower(pidOut, 0);
        }
    }

    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }
        if (!controller.onTarget()) {
            _onTargetSince = 0;
        }

        if(System.currentTimeMillis() >= _endTimeMillis){
            debug("AutoAlign timed out after " + _timeout + "ms at " + imu.getYaw());
            return true;
        }

        if (controller.onTarget()) {
            if (_onTargetSince == 0) {
                info("AutoAlign reached target " + imu.getYaw());
                _onTargetSince = System.currentTimeMillis();
            }

            if (System.currentTimeMillis() > _onTargetSince + Constants.Auto.Align.STEADY_TIME) {
                info("AutoAlign complete after " + Constants.Auto.Align.STEADY_TIME + " at " + imu.getYaw());
                return  true;
            }
        }

        return false;
    }

    @Override
    protected void end() {
        driveTrain.setPower(0,0, true);
        RioLogger.info(this.getClass().getSimpleName(), "AutoAlign finished: angle = " + imu.getYaw() + ", time = " + (System.currentTimeMillis() - startTimeMillis));
        controller.disable();
        RioLogger.debug(this.getClass().getSimpleName(), ("AutoAlign.end() controller disabled"));
    }

    @Override
    public void pidWrite(double output) {
        pidOut = output;
        actOnPidOut();
        //SmartDashboard.putNumber("AutoAlign/pidOut", pidOut);
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    /*
    Used in AutoAlign to select which types of turns to do.
    bothSides will turn normally, or "in place" (hah!)
    leftOnly will use the pidOut to drive the left side of the drivetrain, but the right side will be in 0in/s talon velocity pid mode.
    rightOnly will use the pidOut to drive the right side of the drivetrain, but the left side will be in 0in/s talon velocity pid mode.
     */
    public enum DriveTrainBehavior {
        bothSides,
        leftOnly,
        rightOnly
    }


}
