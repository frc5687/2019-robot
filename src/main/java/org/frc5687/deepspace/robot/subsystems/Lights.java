package org.frc5687.deepspace.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveLights;

public class Lights extends OutliersSubsystem {


    private Robot _robot;
    private Spark _top;
    private DriverStation.Alliance _alliance;

    private double _mainColor;
    private int _blink;

    public Lights(Robot robot) {
        _robot = robot;
        _top = new Spark(RobotMap.PWM.Blinkin);
    }

    public void initialize() {
        _alliance = DriverStation.getInstance().getAlliance();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveLights(_robot.getLights()));
    }

    public void setTop(double val) {
        // DriverStation.reportError("Setting left to " + val, false);
        _top.set(val);
    }
    public void setColor(double color, int blink) {
        _mainColor = color;
        _blink = blink;
    }

    public void setColors(long cycle) {
        if (_blink==0) {
            setTop(_mainColor);
        } else if ((cycle % _blink) == 0) {
            setTop(_mainColor);
        } else {
            setTop(0);
        }
    }

    @Override
    public void updateDashboard() {
        metric("MainColor", _mainColor);
        metric("Blink", _blink);
    }

}
