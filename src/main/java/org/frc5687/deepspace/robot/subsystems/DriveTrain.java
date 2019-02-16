package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.Drive;
import org.frc5687.deepspace.robot.utils.IRDistanceSensor;

import static org.frc5687.deepspace.robot.utils.Helpers.applySensitivityFactor;
import static org.frc5687.deepspace.robot.utils.Helpers.limit;

public class DriveTrain extends OutliersSubsystem {
    private CANSparkMax _leftMaster;
    private CANSparkMax _rightMaster;

    private CANSparkMax _leftFollower;
    private CANSparkMax _rightFollower;

    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;

    private Encoder _leftMagEncoder;
    private Encoder _rightMagEncoder;

    private IRDistanceSensor _frontDistance;

    private OI _oi;

    private double _leftOffset;
    private double _rightOffset;

    private double _oldLeftSpeed;
    private double _oldRightSpeed;
    private boolean _isPaused = false;

    private Shifter _shifter;

    public DriveTrain(Robot robot) {
        info("Constructing DriveTrain class.");
        _oi = robot.getOI();
        _shifter = robot.getShifter();

        _frontDistance = new IRDistanceSensor(RobotMap.Analog.FRONT_IR, IRDistanceSensor.Type.MEDIUM);

        try {
            debug("Allocating motor controllers");
            _leftMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _leftFollower = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_LEFT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightFollower = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_RIGHT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);


            _leftMaster.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
            _leftFollower.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
            _rightMaster.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
            _rightFollower.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);

            debug("Configuring followers");
            _leftFollower.follow(_leftMaster);
            _rightFollower.follow(_rightMaster);

            debug("Configuring encoders");
            _leftEncoder = _leftMaster.getEncoder();
            _rightEncoder = _rightMaster.getEncoder();

        } catch (Exception e) {
            error("Exception allocating drive motor controllers: " + e.getMessage());
        }

        debug("Configuring mag encoders");
        _leftMagEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_A, RobotMap.DIO.DRIVE_LEFT_B);
        _rightMagEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_A, RobotMap.DIO.DRIVE_RIGHT_B);

    }

    private boolean assertMotorControllers() {
        if (_leftMaster==null) { return false; }
        if (_rightMaster==null) { return false; }
        return true;
    }

    @Override
    public void updateDashboard() {
        metric("Front/Value", _frontDistance.getValue());
        metric("Front/Voltage", _frontDistance.getVoltage());
        metric("Front/PID", _frontDistance.pidGet());
        metric("Front/AverageValue", _frontDistance.getAverageValue());
        metric("Front/AverageVoltage", _frontDistance.getAverageVoltage());
        metric("Front/Value", _frontDistance.getValue());
        metric("Front/Inches", _frontDistance.getDistance());
        metric("Neo/Ticks/Left", getLeftTicks());
        metric("Neo/Ticks/Right", getRightTicks());
        metric("Neo/Distance/Left", getLeftDistance());
        metric("Neo/Distance/Right", getRightDistance());
        metric("Neo/Distance/Total", getDistance());
        metric("Mag/Ticks/Left", _leftMagEncoder.get());
        metric("Mag/Ticks/Right", _rightMagEncoder.get());

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(this, _oi));
    }

    public void cheesyDrive(double speed, double rotation) {
        if (!assertMotorControllers()) { return; }
        metric("Speed", speed);
        metric("Rotation", rotation);

        speed = limit(speed, 1);
        //Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation, 1);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed<Constants.DriveTrain.DEADBAND) {
            metric("Rot/Raw", rotation);
            rotation = applySensitivityFactor(rotation, _shifter.getGear()== Shifter.Gear.HIGH  ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);
            metric("Rot/Transformed", rotation);
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
            metric("Rot/LeftMotor", leftMotorOutput);
            metric("Rot/RightMotor", rightMotorOutput);
        } else {
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            speed = applySensitivityFactor(speed, Constants.DriveTrain.SPEED_SENSITIVITY);
            rotation = applySensitivityFactor(rotation, _shifter.getGear()== Shifter.Gear.HIGH  ? Constants.DriveTrain.TURNING_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.TURNING_SENSITIVITY_LOW_GEAR);
            double delta = rotation * Math.abs(speed);
            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }


    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
//         if (!assertMotorControllers()) { return; }
        if (_isPaused == true) {
            leftSpeed = 0;
            rightSpeed = 0;
        }
        try {
            _leftMaster.set(leftSpeed);
            _rightMaster.set(rightSpeed);
        } catch (Exception e) {
            error("DriveTrain.setPower exception: " + e.toString());
        }
        metric("Power/Right", rightSpeed);
        metric("Power/Left", leftSpeed);
    }
    public void pauseMotors() {
        _oldLeftSpeed = _leftMaster.get();
        _oldRightSpeed = _rightMaster.get();
        _leftMaster.set(0);
        _rightMaster.set(0);
        _isPaused = true;
    }

    public void resumeMotors() {
        _leftMaster.set(_oldLeftSpeed);
        _rightMaster.set(_oldRightSpeed);
        _isPaused = false;
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
