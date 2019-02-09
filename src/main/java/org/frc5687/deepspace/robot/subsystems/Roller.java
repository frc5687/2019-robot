package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveRoller;
import org.frc5687.deepspace.robot.commands.HoldRoller;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Roller extends OutliersSubsystem {
    private CANSparkMax _roller;
    private Robot _robot;
    private AnalogInput _ballIR;


    public Roller(Robot robot) {
        _robot = robot;
        try {
            _roller = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _roller.setInverted(Constants.Roller.MOTOR_INVERTED);
        } catch (Exception e) {
            error("Unable to allocate roller controller: " + e.getMessage());
        }
        _ballIR = new AnalogInput(RobotMap.Analog.BALL_IR);
    }
    @Override
    public void updateDashboard() {
        metric("IRValue", _ballIR.getValue());
        metric ("BallDetected", isBallDetected());
    }

    public void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Roller.MAX_SPEED);
        metric("Speed", speed);
        if(_roller==null) { return; }
        _roller.set(speed);
    }
    public boolean isBallDetected() {
        if (_ballIR.getValue() > 2200) {
            return true;
        }
        return false;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveRoller(_robot, this));
    }
}
