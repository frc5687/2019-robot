package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveGobbler;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Gobbler extends OutliersSubsystem {

    private CANSparkMax _intakeGobbler;
    private CANSparkMax _driveGobbler;

    private Robot _robot;
    private IntakeState _intakeState = IntakeState.HOLD;
    public Gobbler(Robot robot) {
        _robot = robot;

        _intakeGobbler = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        _driveGobbler = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_DRIVE, CANSparkMaxLowLevel.MotorType.kBrushless);

    }

    public void setSpeeds(double speed) {
        speed = Helpers.limit(speed, Constants.Gobbler.MAX_DRIVE_SPEED);

        metric("Gobber at " + speed, false);
        _driveGobbler.set(speed);
    }
    public void run(double speed) {
        speed = Helpers.limit(speed, Constants.Gobbler.MAX_INTAKE_SPEED);
        _intakeGobbler.set(speed);
    }
    public void runIntake(double speed) {
        switch(_intakeState) {
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
    public void setIntakeState(IntakeState intakeState) {
        _intakeState = intakeState;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveGobbler(_robot, this));
    }

    @Override
    public void updateDashboard() {

    }
    public enum IntakeState {
        HOLD(0),
        INTAKE(1),
        EJECT(2);

        private int _value;

        IntakeState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }
}
