package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveGobbler;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Gobbler extends OutliersSubsystem {

    private CANSparkMax _roller;
    private CANSparkMax _arm;

    private Robot _robot;
    private GobblerState _gobblerState = GobblerState.HOLD;
    public Gobbler(Robot robot) {
        _robot = robot;

        _roller = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _arm = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);

    }

    public void setSpeeds(double speed) {
        speed = Helpers.limit(speed, Constants.Gobbler.MAX_DRIVE_SPEED);

        metric("Gobber at " + speed, false);
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
}
