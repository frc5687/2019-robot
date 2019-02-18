package org.frc5687.deepspace.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.Drive;
import org.frc5687.deepspace.robot.commands.DriveTalons;

public class Talons extends OutliersSubsystem{
    private Robot _robot;
    private DoubleSolenoid _talonSolenoid;
    private OI _oi;

    public Talons (Robot robot) {
        _robot = robot;
        _talonSolenoid = new DoubleSolenoid(RobotMap.PCM.TALON_OPEN, RobotMap.PCM.TALON_CLOSE);
    }

    public void open() {
        _talonSolenoid.set (DoubleSolenoid.Value.kForward);
        metric("Talon","Open");
    }

    public void close(){
        _talonSolenoid.set(DoubleSolenoid.Value.kReverse);
        metric("Talon","Closed");
    }

    @Override
    public void updateDashboard(){
    }
    @Override
    protected void initDefaultCommand() { setDefaultCommand(new DriveTalons(_robot.getTalons()));
    }

}
