package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveArm;
import org.frc5687.deepspace.robot.utils.HallEffect;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Arm extends OutliersSubsystem implements PIDSource {

    private CANSparkMax _arm;

    private CANEncoder _shoulderEncoder;
    
    private HallEffect _lowHall;
    private HallEffect _intakeHall;
    private HallEffect _secureHall;
    private HallEffect _stowedHall;

    private PIDController _pidController;
    private PIDSourceType _pidSource;
    // PIDController
    // Arm needs to implement PIDSource
    // Amr's pidGet should return encoder position

    // Need private double _pidOut
    private double _pidOut;
    private Robot _robot;
    public Arm(Robot robot) {
        _robot = robot;

        try {
            _arm = new CANSparkMax(RobotMap.CAN.SPARKMAX.ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
            _arm.setInverted(Constants.Arm.MOTOR_INVERTED);
            _arm.setSmartCurrentLimit(Constants.Arm.SHOULDER_STALL_LIMIT, Constants.Arm.SHOULDER_FREE_LIMIT);

            _shoulderEncoder = _arm.getEncoder();
        } catch (Exception e) {
            error("Unable to allocate arm controller: " + e.getMessage());
        }

        _lowHall = new HallEffect(RobotMap.DIO.ARM_LOW_HALL);
        _intakeHall = new HallEffect(RobotMap.DIO.ARM_INTAKE_HALL);
        _secureHall = new HallEffect(RobotMap.DIO.ARM_SECURE_HALL);
        _stowedHall = new HallEffect(RobotMap.DIO.ARM_STOWED_HALL);





        _pidController = new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD, this, new PIDListener());
        // Create PIDController
        // pass in range and PID constants
        // pass this as source and PIDListener as listener
    }

    public void enableBrakeMode() {
        if (_arm==null) { return; }
        _arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_arm==null) { return; }
        _arm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setSpeed(double speed) {
        if (_arm==null) { return; }
        speed = Helpers.limit(speed, Constants.Arm.MAX_DRIVE_SPEED);

        metric("Speed", speed);
        _arm.set(speed);
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
        if (_arm==null) { return; }
        _arm.set(speed);
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
        metric("Encoder", _shoulderEncoder.getPosition());
    }

    public boolean isStowed() { return _stowedHall.get(); }

    public boolean isIntake() { return _intakeHall.get(); }

    public boolean isSecured() { return _secureHall.get(); }

    public boolean isLow() { return _lowHall.get(); }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        _pidSource = pidSource;
    }
    @Override
    public PIDSourceType getPIDSourceType() {
        return null;
    }

    @Override
    public double pidGet() {
        return _shoulderEncoder.getPosition();
    }

    public void setSetpoint(double setPoint){
        _pidController.setSetpoint(setPoint);
        _pidController.enable();
    }
    public PIDController getPIDController(){
        return _pidController;
    }

    // Need setSetPoint(double)
    // set pidcontroller setpoint to param
    // enable

    public void disable(){
        _pidController.disable();
        _arm.set(0);
    }

    // Need disable()
    // Simply disable controller
    // also set speed to 0

    // Need a private PIDListener class
    // pidOut should set _pidout

    // you'll also need:
    // MoveArmToSetPoint command
    //   initialize will call arm.setSetPoint
    //   isFinished will return true if arm pidcontroller is onTarget
    //   end does nada

    // HoldArm command
    //   Does nothing for now
    private class PIDListener implements PIDOutput {

        private double value;

        public double get() {
            return value;
        }

        @Override
        public void pidWrite(double output) {
            value = output;
        }

    }
}


