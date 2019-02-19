package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.stilt.DriveStilt;
import org.frc5687.deepspace.robot.utils.HallEffect;
import org.frc5687.deepspace.robot.utils.IRDistanceSensor;
import static org.frc5687.deepspace.robot.Constants.Stilt.*;
import static org.frc5687.deepspace.robot.utils.Helpers.*;

public class Stilt extends OutliersSubsystem {

    private CANSparkMax _stilt;
    private CANEncoder _neoStiltEncoder;
    private Robot _robot;

    private HallEffect _topHall = new HallEffect(RobotMap.DIO.STILT_HIGH);
    private HallEffect _bottomHall = new HallEffect(RobotMap.DIO.STILT_LOW);

    private IRDistanceSensor _downIR = new IRDistanceSensor(RobotMap.Analog.DOWN_IR, IRDistanceSensor.Type.SHORT);

    public Stilt(Robot robot) {
        _robot = robot;

        try {
            _stilt = new CANSparkMax(RobotMap.CAN.SPARKMAX.STILT, CANSparkMaxLowLevel.MotorType.kBrushless);
            _stilt.setInverted(MOTOR_INVERTED);
            _neoStiltEncoder = _stilt.getEncoder();
        }catch (Exception e) {
            error("Unable to allocate stilt controller: " + e.getMessage());
        }
    }

    public void drive(double desiredSpeed) {
        double speed = limit(desiredSpeed, isAtBottom() ? 0 : -MAX_DOWN_SPEED , isAtTop() ? 0 : MAX_UP_SPEED);
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
        _stilt.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public double getRawNeoEncoder() {
        if (_stilt==null) { return 0; }
        return _neoStiltEncoder.getPosition();
    }

    public boolean isAtTop() {
        return _topHall.get();
    }

    public boolean isAtBottom() {
        return _bottomHall.get();
    }

    public boolean isOnSurface() {
        return false;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveStilt(this, _robot.getOI()));
    }
    @Override
    public void updateDashboard() {
        metric("NEOEncoder", getRawNeoEncoder());
        metric("AtTop", isAtTop());
        metric("AtBottom", isAtBottom());
        metric("IRValue", _downIR.getAverageValue());
        metric("IRVoltage", _downIR.getAverageVoltage());
    }


}