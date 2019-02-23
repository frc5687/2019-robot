package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.intake.RunIntake;
import org.frc5687.deepspace.robot.utils.Helpers;

import static org.frc5687.deepspace.robot.Constants.Intake.*;

public class Intake extends OutliersSubsystem {


    private Robot _robot;
    private DoubleSolenoid _wristSolenoid;
    private DoubleSolenoid _clawSolenoid;
    private TalonSRX _roller;
    private RollerMode _rollerMode;
    private boolean _forceOn;
    private AnalogInput _ballIR;

    public Intake (Robot robot) {
        _robot = robot;
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.WRIST_UP, RobotMap.PCM.WRIST_DOWN);
        _clawSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_OPEN, RobotMap.PCM.CLAW_CLOSE);
        try {
            _roller = new TalonSRX(RobotMap.CAN.TALONSRX.ROLLER);
            _roller.configPeakOutputForward(HIGH_POW, 0);
            _roller.configPeakOutputReverse(LOW_POW,0);
            _roller.configNominalOutputForward(0.0, 0);
            _roller.configNominalOutputReverse(0.0, 0);
            _roller.setInverted(Constants.Intake.MOTOR_INVERTED);
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

    public void startRoller() {
        setRollerSpeed(ROLLER_SPEED);
        _forceOn = true;
        metric("ForceOn", _forceOn);
    }

    public void stopRoller() {
        setRollerSpeed(0);
        _forceOn = false;
        metric("ForceOn", _forceOn);
    }

    public void gripClaw(){
         _clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void pointClaw() {
        _clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void raiseWrist() {
        _wristSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void lowerWrist() {
        _wristSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void releaseWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kOff); }

    private void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Intake.MAX_ROLLER_SPEED);
        metric("Speed", speed);
        if (_roller == null) {
            return;
        }
        run(speed);
    }

    public void run(double speed) {
        if (_forceOn) { speed = ROLLER_SPEED; }
        metric("RollerSpeed", speed);
        _roller.set(ControlMode.PercentOutput, speed);
    }

    public boolean isBallDetected() { return _ballIR.getValue() > Constants.Intake.CARGO_DETECTED_THRESHOLD;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new RunIntake(_robot, this));
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

