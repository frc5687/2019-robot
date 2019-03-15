package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveArm;
import org.frc5687.deepspace.robot.utils.HallEffect;
import static org.frc5687.deepspace.robot.Constants.Arm.*;
import static org.frc5687.deepspace.robot.utils.Helpers.*;

public class Arm extends OutliersSubsystem implements PIDSource {

    private CANSparkMax _leftSpark;
    private CANSparkMax _rightSpark;

    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;

    private HallEffect _rightStowedhall;
    private HallEffect _leftStowedhall;
    // private HallEffect _rightLowhall;
    // private HallEffect _leftLowhall;


    private double _leftOffset = 0;
    private double _rightOffset = 0;

    private double _lastSpeed = 0;
    // Need private double _pidOut
    private double _pidOut;
    private Robot _robot;
    public Arm(Robot robot) {
        _robot = robot;

        try {
            _leftSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);


            _leftSpark.setInverted(Constants.Arm.LEFT_MOTOR_INVERTED);
            _rightSpark.setInverted(Constants.Arm.RIGHT_MOTOR_INVERTED);

            _leftSpark.setSmartCurrentLimit(Constants.Arm.SHOULDER_STALL_LIMIT, Constants.Arm.SHOULDER_FREE_LIMIT);
            _rightSpark.setSmartCurrentLimit(Constants.Arm.SHOULDER_STALL_LIMIT, Constants.Arm.SHOULDER_FREE_LIMIT);

            _leftEncoder = _leftSpark.getEncoder();
            _rightEncoder  = _rightSpark.getEncoder();
        } catch (Exception e) {
            error("Unable to allocate arm controller: " + e.getMessage());
        }

        _rightStowedhall = new HallEffect(RobotMap.DIO.ARM_RIGHT_STOWED_HALL);
        _leftStowedhall = new HallEffect(RobotMap.DIO.ARM_LEFT_STOWED_HALL);
        // _rightLowhall = new HallEffect(RobotMap.DIO.ARM_RIGHT_LOW_HALL);
        // _leftLowhall = new HallEffect(RobotMap.DIO.ARM_LEFT_LOW_HALL);
    }

    public void enableBrakeMode() {
        if (_leftSpark ==null) { return; }
        _leftSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _rightSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_leftSpark ==null) { return; }
        _leftSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setSpeed(double speed) {
        if (_leftSpark == null || _rightSpark==null) { return; }
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }


    public void setLeftSpeed(double speed) {
        if (_leftSpark == null) { return; }
        speed = limit(speed,
                isLeftStowed() ? 0 : -MAX_DRIVE_SPEED ,
                isLeftLow() ? HOLD_SPEED : MAX_DRIVE_SPEED);
        metric("LeftSpeed", speed);
        if(isLeftStowed()) { resetLeftEncoder(); }
        _leftSpark.set(speed);
    }

    public void setRightSpeed(double speed) {
        if (_rightSpark == null) { return; }
        speed = limit(speed,
                isRightStowed() ? 0 : -MAX_DRIVE_SPEED ,
                isRightLow() ? HOLD_SPEED : MAX_DRIVE_SPEED);
        metric("RightSpeed", speed);
        if(isRightStowed()) { resetRightEncoder(); }
        _rightSpark.set(speed);
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveArm(_robot, this));
    }

    @Override
    public void updateDashboard() {
//        metric("LowRightHall", _rightLowhall.get());
//        metric("LowLeftHall", _leftLowhall.get());
        metric("StowedRightHall", _rightStowedhall.get());
        metric("StowedLeftHall", _leftStowedhall.get());
        metric("LeftEncoder", getLeftPosition());
        metric("RightEncoder", getRightPosition());
        metric("Position", getPosition());
        metric("Angle", getAngle());
    }

    public boolean isStowed() {
        return isLeftStowed() || isRightStowed();

    }

    public boolean isRightStowed() {
        return _rightStowedhall.get();

    }

    public boolean isLeftStowed() {
        return _leftStowedhall.get();

    }

    public boolean isRightLow() {
        return false;
    }

    public boolean isLeftLow() {
        return false;
    }


    public boolean isLow() {
        return isLeftLow() || isRightLow();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getPosition();
    }

    public double getPosition() {
        return (getLeftPosition() + getRightPosition())/2;
    }

    public double getLeftPosition() {
        return _leftEncoder.getPosition() - _leftOffset;
    }

    public double getRightPosition() {
        return _rightEncoder.getPosition() - _rightOffset;
    }

    public void resetEncoders() {
        resetLeftEncoder();
        resetRightEncoder();
    }

    public void resetLeftEncoder() {
        _leftOffset = _leftEncoder.getPosition();
    }

    public void resetRightEncoder() {
        _rightOffset = _rightEncoder.getPosition();
    }

    public void resetEncoder() {
        //_offset = _shoulderEncoder.getPosition();
        //DriverStation.reportError("Resetting arm offset to " + _offset, false);
    }
        public double getAngle () {
            return Constants.Arm.STOWED_ANGLE + (getPosition() * Constants.Arm.DEGREES_PER_TICK);
        }


        public enum HallEffectSensor {
            LOW,
            INTAKE,
            SECURE,
            STOWED
        }

        public enum Setpoint {
            Stowed(0),
            Secure(45),
            Intake(80),
            Handoff(100),
            Floor(110),
            Climb(120);

            private int _value;

            Setpoint(int value) {
                this._value = value;
            }

            public int getValue() {
                return _value;
            }

            public int getPosition() {
                return _value;
            }

        }

    public enum MotionMode {
        HallOnly(0),
        Simple(1),
        PID(2),
        Path(3);

        private int _value;

        MotionMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }
}



