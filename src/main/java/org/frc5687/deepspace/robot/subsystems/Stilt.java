package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveStilt;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Stilt extends OutliersSubsystem {

    private CANSparkMax _stilt;
    private CANEncoder _neoStiltEncoder;
    private Robot _robot;

    public Stilt(Robot robot) {
        _robot = robot;

        try {
            _stilt = new CANSparkMax(RobotMap.CAN.SPARKMAX.STILT, CANSparkMaxLowLevel.MotorType.kBrushless);
            _stilt.setInverted(Constants.Stilt.MOTOR_INVERTED);
            _neoStiltEncoder = _stilt.getEncoder();
        }catch (Exception e) {
            error("Unable to allocate stilt controller: " + e.getMessage());
        }
    }

    public void drive(double desiredSpeed) {
        double speed = desiredSpeed;
        metric("rawSpeed", desiredSpeed);
        metric("speed", speed);
        _stilt.set(speed);
    }

    public void enableBrakeMode() {
        if (_stilt==null) { return; }
        _stilt.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_stilt==null) { return; }
        // _stlit.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public double getRawNeoEncoder() {
        if (_stilt==null) { return 0; }
        return _neoStiltEncoder.getPosition();
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveStilt(_robot, this));
    }
    @Override
    public void updateDashboard() {
        metric("NEOEncoder", getRawNeoEncoder());

    }


}