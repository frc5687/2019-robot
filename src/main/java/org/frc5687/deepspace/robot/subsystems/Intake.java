package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveRoller;
import org.frc5687.deepspace.robot.commands.RunIntake;
import org.frc5687.deepspace.robot.utils.Helpers;

import static org.frc5687.deepspace.robot.Constants.Roller.INTAKE_SPEED;

public class Intake extends OutliersSubsystem {

    private Robot _robot;
    private DoubleSolenoid _wristSolenoid;
    private DoubleSolenoid _talonsSolenoid;
    private TalonSRX _roller;
    private boolean _forceOn;

    public Intake (Robot robot) {
        _robot = robot;
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.WRIST_UP,RobotMap.PCM.WRIST_DOWN);
        _talonsSolenoid = new DoubleSolenoid(RobotMap.PCM.TALON_OPEN, RobotMap.PCM.TALON_CLOSE);
        try {
            _roller = new TalonSRX(RobotMap.CAN.TALONSRX.ROLLER);
            _roller.setInverted(Constants.Roller.MOTOR_INVERTED);
        } catch (Exception e) {
            error("Unable to allocate roller controller: " + e.getMessage());
        }
    }

    @Override
    public void updateDashboard() {
    }

    public void start() {
        _forceOn = true;
        metric("ForceOn", _forceOn);
    }

    public void stop() {
        _forceOn = false;
        metric("ForceOn", _forceOn);
    }

    public void setSpeed(double speed) {
        setRollerSpeed(_forceOn ? INTAKE_SPEED : speed);
    }

    private void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Roller.MAX_SPEED);
        metric("Speed", speed);

        if(_roller==null) { return; }
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new RunIntake(_robot, this));
    }


}
