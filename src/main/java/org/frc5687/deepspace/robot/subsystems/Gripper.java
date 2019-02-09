package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DropBall;
import org.frc5687.deepspace.robot.commands.WristRelease;

public class Gripper extends OutliersSubsystem{
    private Robot _robot;
    private CANSparkMax _vacuumFan;

    public Gripper(Robot robot){
        debug("Finding vacuum motor.");
        try{
            _vacuumFan = new CANSparkMax(RobotMap.CAN.SPARKMAX.INTAKE_VACUUM, CANSparkMaxLowLevel.MotorType.kBrushless);
        } catch (Exception e){
            error("Exception allocating vacuum motor controller: " + e.getMessage());
            return;
        }
    }
    public void suckBall(){

        _vacuumFan.set(Constants.Gripper.VACUUM_SPEED);
    }
    public void dropBall(){
        _vacuumFan.set(0);
    }



    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DropBall(this));
    }
    @Override
    public void updateDashboard() {
    }
}