package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveRoller;
import org.frc5687.deepspace.robot.commands.RunIntake;
import org.frc5687.deepspace.robot.utils.Helpers;

import static org.frc5687.deepspace.robot.Constants.Intake.*;
import static org.frc5687.deepspace.robot.Constants.Roller.INTAKE_SPEED;

public class Intake extends OutliersSubsystem {


    private Robot _robot;
    private DoubleSolenoid _wristSolenoid;
    private DoubleSolenoid _talonsSolenoid;
    private TalonSRX _roller;
    private boolean _forceOn;
    private double _speed;

    public Intake (Robot robot) {
        _robot = robot;
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.WRIST_UP, RobotMap.PCM.WRIST_DOWN);
        _talonsSolenoid = new DoubleSolenoid(RobotMap.PCM.TALONS_OPEN, RobotMap.PCM.TALONS_CLOSE);
        try {
            _roller = new TalonSRX(RobotMap.CAN.TALONSRX.ROLLER);
            _roller.configPeakOutputForward(HIGH_POW, 0);
            _roller.configPeakOutputReverse(LOW_POW,0);
            _roller.configNominalOutputForward(0.0, 0);
            _roller.configNominalOutputReverse(0.0, 0);
            _roller.setInverted(Constants.Roller.MOTOR_INVERTED);
        } catch (Exception e) {
            error("Unable to allocate roller controller: " + e.getMessage());
        }
    }

    @Override
    public void updateDashboard() {
    }

    public void startRollers() {
        setRollerSpeed(ROLLER_SPEED);
        metric("ForceOn", _forceOn);
    }

    public void stopRoller() {
        setRollerSpeed(0);
        metric("ForceOn", _forceOn);
    }
     public void openTalons(){
         _talonsSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void closeTalons() {
        _talonsSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void raiseWrist() {
        _wristSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void lowerWrist() {
        _wristSolenoid.set(DoubleSolenoid.Value.kForward);
    }


    public void setSpeed(double speed) {
        setRollerSpeed(speed);
    }

    private void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Roller.MAX_SPEED);
        metric("Speed", speed);
        if (_roller == null) {
            return;
        }
        speed = _speed;
        _roller.set(ControlMode.PercentOutput, speed);
    }
    public void run() {
        _roller.set(ControlMode.PercentOutput, _speed);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new RunIntake(_robot, this));
    }


}

