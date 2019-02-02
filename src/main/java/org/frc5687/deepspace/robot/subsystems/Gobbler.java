package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalSource;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveGobbler;
import org.frc5687.deepspace.robot.utils.HallEffect;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Gobbler extends OutliersSubsystem {

    private CANSparkMax _roller;
    private CANSparkMax _arm;

    private CANEncoder _shoulderEncoder;
    
    private HallEffect _lowHall;
    private HallEffect _intakeHall;
    private HallEffect _secureHall;
    private HallEffect _stowedHall;


    private Robot _robot;
    private GobblerState _gobblerState = GobblerState.HOLD;
    public Gobbler(Robot robot) {
        _robot = robot;

        _roller = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _arm = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
        _roller.setInverted(Constants.Gobbler.ROLLER_MOTOR_INVERTED);
        _arm.setInverted(Constants.Gobbler.ARM_MOTOR_INVERTED);


        _shoulderEncoder = _arm.getEncoder();

        _lowHall = new HallEffect(RobotMap.DIO.GOBBLER_LOW_HALL);
        _intakeHall = new HallEffect(RobotMap.DIO.GOBBLER_INTAKE_HALL);
        _secureHall = new HallEffect(RobotMap.DIO.GOBBLER_SECURE_HALL);
        _stowedHall = new HallEffect(RobotMap.DIO.GOBBLER_STOWED_HALL);

    }

    public void setSpeeds(double speed) {
        speed = Helpers.limit(speed, Constants.Gobbler.MAX_DRIVE_SPEED);

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
        _arm.set(speed);
    }
    public void run(double speed) {
        speed = Helpers.limit(speed, Constants.Gobbler.MAX_INTAKE_SPEED);
        _roller.set(speed);
    }

    public void runGobbler(double speed) {
        switch(_gobblerState) {
            case HOLD:
                run(Constants.Gobbler.HOLD_SPEED);
                break;
            case INTAKE:
                run(Constants.Gobbler.INTAKE_SPEED);
                break;
            case EJECT:
                run(speed);
                break;
            default:
                run(0);
        }
    }

    public void setGobblerState(GobblerState gobblerState) {
        _gobblerState = gobblerState;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveGobbler(_robot, this));
    }

    @Override
    public void updateDashboard() {
        metric("LowHall", _lowHall.get());
        metric("IntakeHall", _intakeHall.get());
        metric("SecureHall", _secureHall.get());
        metric("StowedHall", _stowedHall.get());
        metric("Encoder", _shoulderEncoder.getPosition());
    }

    public enum GobblerState {
        HOLD(0),
        INTAKE(1),
        EJECT(2);

        private int _value;

        GobblerState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }
    public boolean isStowed() { return _stowedHall.get(); }

    public boolean isIntake() { return _intakeHall.get(); }

    public boolean isSecured() { return _secureHall.get(); }

    public boolean isLow() { return _lowHall.get(); }
}
