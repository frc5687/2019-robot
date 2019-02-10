package org.frc5687.deepspace.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.Drive;
import org.frc5687.deepspace.robot.commands.DriveSpear;

public class Spear extends OutliersSubsystem{
    private Robot _robot;
    private DoubleSolenoid _spearSolenoid;
    private OI _oi;
    public Spear (Robot robot){
        _robot = robot;
       _spearSolenoid = new DoubleSolenoid(RobotMap.PCM.SPEAR_OPEN, RobotMap.PCM.SPEAR_CLOSE);
    }

    public void open(){
        _spearSolenoid.set(DoubleSolenoid.Value.kForward);
        metric("Spear","Open");
    }
    public void close(){
        _spearSolenoid.set(DoubleSolenoid.Value.kReverse);
        metric("Spear","Closed");
    }

    @Override
    public void updateDashboard() {
    }
    @Override
    protected void initDefaultCommand() {
            setDefaultCommand(new DriveSpear(_robot.getSpear()));
    }
}
