package org.frc5687.deepspace.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;


public class AutoAlignToTarget extends OutliersCommand implements PIDOutput {

    private PIDController controller;
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

    NetworkTable _table;

    private OI _oi;

    private DriveTrainBehavior _driveTrainBehavior = DriveTrainBehavior.bothSides;

    private double _tolerance;

    public AutoAlignToTarget(Robot robot, double speed, long timeout, double tolerance) {
        this(robot.getDriveTrain(), robot.getIMU(), speed, timeout, tolerance, "");
        _oi = robot.getOI();
    }


    public AutoAlignToTarget(DriveTrain driveTrain, AHRS imu, double speed, long timeout, double tolerance, String message) {
        requires(driveTrain);
        this.angle = angle;
        this.speed = speed;
        this.driveTrain = driveTrain;
        this.imu = imu;
        _timeout = timeout;
        _tolerance = tolerance;
        _message = message;
    }

    @Override
    protected void initialize() {
        double kP = Constants.Auto.AlignToTarget.kP; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kI = Constants.Auto.AlignToTarget.kI; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kD = Constants.Auto.AlignToTarget.kD; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        // 1: Read current target angle from limelight
        // 2: Read current yaw from navX
        // 3: Set controller.angle to sum
        _table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = _table.getEntry("tx");

        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = imu.getAngle();
        angle = limeLightAngle + yawAngle;

        metric("LimeLightAngle", limeLightAngle);
        metric("YawAngle", yawAngle);
        metric("angle", angle);

        controller = new PIDController(kP, kI, kD, imu, this, 0.01);
        controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        controller.setOutputRange(-speed, speed);
        controller.setAbsoluteTolerance(_tolerance);
        controller.setContinuous();
        controller.setSetpoint(angle);
        controller.enable();
        metric("setpoint", angle);
        error("AutoAlign " + _message + " initialized to " + angle + " at " + speed);
        error("kP="+kP+" , kI="+kI+", kD="+kD + ",T="+ Constants.Auto.AlignToTarget.TOLERANCE);
        startTimeMillis = System.currentTimeMillis();
        _endTimeMillis = startTimeMillis + _timeout;


    }

    @Override
    protected void execute() {
        NetworkTableEntry tx = _table.getEntry("tx");
        double limeLightAngle = tx.getDouble(0.0);
        double yawAngle = imu.getAngle();
        angle = limeLightAngle + yawAngle;

        metric("startoffset", limeLightAngle);
        metric("startyaw", yawAngle);
        metric("target", angle);

        if (Math.abs(angle - controller.getSetpoint()) > _tolerance) {
            controller.setSetpoint(angle);
            metric("setpoint", angle);
        }
        metric("onTarget", controller.onTarget());
        metric("yaw", imu.getYaw());

        actOnPidOut();


    }

    private void actOnPidOut() {
        if (pidOut > 0 && pidOut < Constants.Auto.AlignToTarget.MINIMUM_SPEED) {
            pidOut = Constants.Auto.AlignToTarget.MINIMUM_SPEED;
        }
        if (pidOut < 0 && pidOut > -Constants.Auto.AlignToTarget.MINIMUM_SPEED) {
            pidOut = -Constants.Auto.AlignToTarget.MINIMUM_SPEED;
        }
        metric("pidOut", pidOut);
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

        if((_oi!=null && !_oi.isAutoTargetPressed()) && System.currentTimeMillis() >= _endTimeMillis){
            error("AutoAlignToTarget timed out after " + _timeout + "ms at " + imu.getYaw());
            return true;
        }

        if (controller.onTarget()) {
            if (_onTargetSince == 0) {
                error("AutoAlignToTarget reached target " + imu.getYaw());
                _onTargetSince = System.currentTimeMillis();
            }

            if ((_oi!=null && !_oi.isAutoTargetPressed()) && System.currentTimeMillis() > _onTargetSince + Constants.Auto.AlignToTarget.STEADY_TIME) {
                error("AutoAlignToTarget complete after " + Constants.Auto.AlignToTarget.STEADY_TIME + " at " + imu.getYaw());
                return  true;
            }
        }

        return false;
    }
    @Override
    protected void end() {
        driveTrain.setPower(0,0, true);
        error("AutoAlign finished: angle = " + imu.getYaw() + ", time = " + (System.currentTimeMillis() - startTimeMillis));
        controller.disable();
        error("AutoAlign.end() controller disabled");

    }
    @Override
    public void pidWrite(double output) {
        pidOut = output;
//        actOnPidOut();

    }
    public enum DriveTrainBehavior {
        bothSides,
        leftOnly,
        rightOnly
    }

}
