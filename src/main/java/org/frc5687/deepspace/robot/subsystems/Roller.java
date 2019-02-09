package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveRoller;
import org.frc5687.deepspace.robot.commands.HoldRoller;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Roller extends OutliersSubsystem {
    private CANSparkMax _roller;
    private Robot _robot;


    public Roller(Robot robot) {
        _robot = robot;
        _roller = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _roller.setInverted(Constants.Roller.MOTOR_INVERTED);

    }
    @Override
    public void updateDashboard() {
    }

    public void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Roller.MAX_SPEED);
        metric("Speed", speed);
        _roller.set(speed);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveRoller(_robot, this));
    }
}
