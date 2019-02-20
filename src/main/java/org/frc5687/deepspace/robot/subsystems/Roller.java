package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveRoller;
import org.frc5687.deepspace.robot.utils.Helpers;
import static org.frc5687.deepspace.robot.Constants.Roller.*;

public class Roller extends OutliersSubsystem {
    private CANSparkMax _roller;
    private Robot _robot;
    private AnalogInput _ballIR;
    private boolean _forceOn;
    private RollerMode _rollerMode;
    private CANEncoder _rollerEncoder;


    public Roller(Robot robot) {
        _robot = robot;
        try {
            _roller = new CANSparkMax(RobotMap.CAN.SPARKMAX.ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rollerEncoder = _roller.getEncoder();
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

    public void start() {
        _forceOn = true;
        metric("ForceOn", _forceOn);
    }

    public void stop() {
        _forceOn = false;
        metric("ForceOn", _forceOn);
    }

    public CANEncoder getRollerEncoder(){
        return _rollerEncoder;
    }
    public double getRollerEncoderPos(){
        return _rollerEncoder.getPosition();
    }

    public void setSpeed(double speed) {
        setRollerSpeed(_forceOn ? INTAKE_SPEED : speed);
    }

    private void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Roller.MAX_SPEED);
        metric("Speed", speed);

        if(_roller==null) { return; }
        _roller.set(speed);
    }

    public boolean isBallDetected() {
        return _ballIR.getValue() > Constants.Roller.CARGO_DETECTED_THRESHOLD;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveRoller(_robot, this));
    }
    public void setRollerMode(RollerMode rollerMode) {
        _rollerMode = rollerMode;
    }
    public RollerMode getRollerMode() {
        return _rollerMode;
    }

    public enum RollerMode {
        RUNNING(0),
        WAITING(1),
        DONE(2);

        private int _value;

        RollerMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }
}
