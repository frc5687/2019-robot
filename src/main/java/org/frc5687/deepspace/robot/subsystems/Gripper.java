package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.StopGripper;

public class Gripper extends OutliersSubsystem{
    private Robot _robot;
    private TalonSRX _vacuumFan;

    public double HIGH_POW = 1.0;
    public double LOW_POW = -HIGH_POW;

    public Gripper(Robot robot){
        debug("Finding vacuum motor.");
        try{
            _vacuumFan = new TalonSRX(RobotMap.CAN.TALONSRX.GRIPPER_VACUUM);
        } catch (Exception e){
            error("Exception allocating vacuum motor controller: " + e.getMessage());
            return;
        }
        _vacuumFan.configPeakOutputForward(HIGH_POW, 0);
        _vacuumFan.configPeakOutputReverse(LOW_POW,0);
        _vacuumFan.configNominalOutputForward(0.0, 0);
        _vacuumFan.configNominalOutputReverse(0.0, 0);
        _vacuumFan.setInverted(Constants.Gripper.MOTOR_INVERTED);
        enableBrakeMode();
    }
    public void enableBrakeMode() {
        try {
            _vacuumFan.setNeutralMode(NeutralMode.Brake);

        } catch (Exception e) {
            error("DriveTrain.enableBrakeMode exception: " + e.toString());
        }
        metric("neutralMode", "Brake");
    }

    public void setSpeed(double speed){

        _vacuumFan.set(ControlMode.PercentOutput, speed);
        metric("Speed", speed);
    }




    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new StopGripper(this));
    }
    @Override
    public void updateDashboard() {

    }
}