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

public class Arm extends OutliersSubsystem implements PIDSource {

    private CANSparkMax _leftSpark;
    private CANSparkMax _rightSpark;

    private CANEncoder _shoulderEncoder;
    
    private HallEffect _lowHall;
    private HallEffect _intakeHall;
    private HallEffect _secureHall;
    private HallEffect _stowedHall;


    private double _offset = 0;

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

            _shoulderEncoder = _leftSpark.getEncoder();
        } catch (Exception e) {
            error("Unable to allocate arm controller: " + e.getMessage());
        }

        _lowHall = new HallEffect(RobotMap.DIO.ARM_LOW_HALL);
        _intakeHall = new HallEffect(RobotMap.DIO.ARM_INTAKE_HALL);
        _secureHall = new HallEffect(RobotMap.DIO.ARM_SECURE_HALL);
        _stowedHall = new HallEffect(RobotMap.DIO.ARM_STOWED_HALL);

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
        if (_leftSpark ==null) { return; }
        // speed = Helpers.limit(speed, Constants.Arm.MAX_DRIVE_SPEED);

        metric("Speed", speed);
        _leftSpark.set(speed);
        _rightSpark.set(speed);
    }

    public void drive(double desiredSpeed, boolean overrideCaps) {
        double speed = desiredSpeed;
        if (!overrideCaps) {
            if (speed > 0 && isStowed()) {
                speed = 0;
            } else if (speed < 0 && isLow()) {
                speed = 0;
            }
        }
        metric("rawSpeed", desiredSpeed);
        metric("speed", speed);
        if (_leftSpark ==null) { return; }
        _leftSpark.set(speed);
        _rightSpark.set(speed);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveArm(_robot, this));
    }

    @Override
    public void updateDashboard() {
        metric("LowHall", _lowHall.get());
        metric("IntakeHall", _intakeHall.get());
        metric("SecureHall", _secureHall.get());
        metric("StowedHall", _stowedHall.get());
        if (_shoulderEncoder==null) { return; }
        metric("Encoder", getPosition());
        metric("Angle", getAngle());
    }

    public boolean isStowed() { return _stowedHall.get(); }

    public boolean isIntake() { return _intakeHall.get(); }

    public boolean isSecured() { return _secureHall.get(); }

    public boolean isLow() { return _lowHall.get(); }

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
        return _shoulderEncoder.getPosition() - _offset;
    }

    public void resetEncoder() {
        _offset = _shoulderEncoder.getPosition();
        DriverStation.reportError("Resetting arm offset to " + _offset , false);
    }

    public double getAngle() {
        return Constants.Arm.STOWED_ANGLE + (_shoulderEncoder.getPosition() * Constants.Arm.DEGREES_PER_TICK);
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


