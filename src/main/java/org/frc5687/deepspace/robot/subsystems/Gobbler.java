package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveGobbler;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Gobbler extends OutliersSubsystem {

    private CANSparkMax _leftGobbler;
    private CANSparkMax _rightGobbler;

    private Robot _robot;
    public Gobbler(Robot robot) {
        _robot = robot;

        _leftGobbler = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        _rightGobbler = new CANSparkMax(RobotMap.CAN.SPARKMAX.GOBBLER_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

    }

    public void setSpeeds(double speed) {
        speed = Helpers.limit(speed, Constants.Gobbler.MAX_INTAKE_SPEED);

        metric("Gobber at " + speed, false);
        _leftGobbler.set(speed);
        _rightGobbler.set(speed);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveGobbler(_robot, this));
    }

    @Override
    public void updateDashboard() {

    }
}
