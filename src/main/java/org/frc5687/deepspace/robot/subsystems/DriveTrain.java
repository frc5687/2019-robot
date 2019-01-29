package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.Drive;

import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;
import static org.frc5687.deepspace.robot.utils.Helpers.limit;

public class DriveTrain extends OutliersSubsystem {
    private CANSparkMax _leftMaster;
    private CANSparkMax _rightMaster;

    private CANSparkMax _leftFollower;
    private CANSparkMax _rightFollower;

    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;

    private OI _oi;

    private double _leftOffset;
    private double _rightOffset;

    public DriveTrain(Robot robot) {
        info("Constructing DriveTrain class.");
        _oi = robot.getOI();

        debug("Allocating motor controllers");
        try {
            _leftMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _leftFollower = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_LEFT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightFollower = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_RIGHT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        } catch (Exception e) {
            error("Exception allocating drive motor controllers: " + e.getMessage());
            return;
        }
        debug("Configuring followers");
        _leftFollower.follow(_leftMaster);
        _rightFollower.follow(_rightMaster);

        debug("Configuring encoders");
        _leftEncoder = _leftMaster.getEncoder();
        _rightEncoder = _rightMaster.getEncoder();
    }

    private boolean assertMotorControllers() {
        if (_leftMaster==null) { return false; }
        if (_rightMaster==null) { return false; }
        return true;
    }

    @Override
    public void updateDashboard() {
        metric("Ticks/Left", getLeftTicks());
        metric("Ticks/Right", getRightTicks());
        metric("Distance/Left", getLeftDistance());
        metric("Distance/Right", getRightDistance());
        metric("Distance/Total", getDistance());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(this, _oi));
    }

    public void cheesyDrive(double speed, double rotation) {
        if (!assertMotorControllers()) { return; }
        metric("Speed", speed);
        metric("Rotation", rotation);

        speed = limit(speed);
        //Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        speed = Math.copySign(speed * speed, speed);
        rotation = applySensitivityFactor(rotation,  true ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed==0.0) {
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
        } else {
            double delta = rotation * Math.abs(speed);
            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }


    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
        if (!assertMotorControllers()) { return; }
        try {
            _leftMaster.set(leftSpeed);
            _rightMaster.set(rightSpeed);
        } catch (Exception e) {
            error("DriveTrain.setPower exception: " + e.toString());
        }
        metric("Power/Right", rightSpeed);
        metric("Power/Left", leftSpeed);
    }


    public double getLeftDistance() {
        if (!assertMotorControllers()) { return 0; }
        return (getLeftTicks()  - _leftOffset) * Constants.DriveTrain.LEFT_RATIO;
    }

    public double getRightDistance() {
        if (!assertMotorControllers()) { return 0; }

        return (getRightTicks() - _rightOffset) * Constants.DriveTrain.RIGHT_RATIO;
    }

    public double getLeftTicks() {
        if (_leftEncoder==null) { return 0; }
        return _leftEncoder.getPosition();
    }


    public double getRightTicks() {
        if (_rightEncoder==null) { return 0; }
        return _rightEncoder.getPosition();
    }

    public double getDistance() {
        if (Math.abs(getRightTicks())<10) {
            return getLeftDistance();
        }
        if (Math.abs(getLeftTicks())<10) {
            return getRightDistance();
        }
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public void resetDriveEncoders() {
        _leftOffset = getLeftTicks();
        _rightOffset = getRightTicks();
    }


}
