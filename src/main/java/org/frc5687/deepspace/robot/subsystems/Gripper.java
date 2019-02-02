package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.WristRelease;

public class Gripper extends OutliersSubsystem{
    private Robot _robot;
    private CANSparkMax _vacuumFan;
    private DoubleSolenoid _vacuumPusher;

    public Gripper(Robot robot){
        _vacuumPusher = new DoubleSolenoid(RobotMap.PCM.VACUUM_PISTON_RELEASE, RobotMap.PCM.VACUUM_PISTON_BACK);
        debug("Finding vacuum motor.");
        try{
            _vacuumFan = new CANSparkMax(RobotMap.CAN.SPARKMAX.INTAKE_VACUUM, CANSparkMaxLowLevel.MotorType.kBrushless);
        } catch (Exception e){
            error("Exception allocating vacuum motor controller: " + e.getMessage());
            return;
        }
    }
    public void suckBall(){

        _vacuumFan.set(Constants.Intake.VACUUM_SPEED);
    }
    public void dropBall(){
        _vacuumFan.set(0);
        _vacuumPusher.set(DoubleSolenoid.Value.kForward);
    }



    @Override
    protected void initDefaultCommand() {
    }
    @Override
    public void updateDashboard() {
    }
}