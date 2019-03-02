package org.frc5687.deepspace.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.stilt.DriveStilt;
import org.frc5687.deepspace.robot.utils.HallEffect;
import org.frc5687.deepspace.robot.utils.IRDistanceSensor;
import static org.frc5687.deepspace.robot.Constants.Stilt.*;
import static org.frc5687.deepspace.robot.utils.Helpers.*;

public class Stilt extends OutliersSubsystem {

    private CANSparkMax _lifterSpark;
    private CANEncoder _neoStiltEncoder;
    private PWMVictorSPX _wheelieVictor;

    private Robot _robot;

    private HallEffect _topHall;
    private HallEffect _middleHall;
    private HallEffect _bottomHall;

    private double _offset = 0;

    private IRDistanceSensor _downIR = new IRDistanceSensor(RobotMap.Analog.DOWN_IR, IRDistanceSensor.Type.SHORT);

    public Stilt(Robot robot) {
        _robot = robot;

        try {
            _lifterSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.STILT, CANSparkMaxLowLevel.MotorType.kBrushless);
            _lifterSpark.setInverted(MOTOR_INVERTED);
            _neoStiltEncoder = _lifterSpark.getEncoder();
            enableBrakeMode();
        }catch (Exception e) {
            error("Unable to allocate stilt controller: " + e.getMessage());
        }

        _topHall = new HallEffect(RobotMap.DIO.STILT_HIGH);
        _middleHall = new HallEffect(RobotMap.DIO.STILT_MIDDLE);
        _bottomHall = new HallEffect(RobotMap.DIO.STILT_LOW);

        _wheelieVictor = new PWMVictorSPX(RobotMap.PWM.Wheelie);
        _wheelieVictor.setInverted(false);
    }

    public void setLifterSpeed(double desiredSpeed) {
        double speed = desiredSpeed; // limit(desiredSpeed, isAtBottom() ? 0 : -MAX_DOWN_SPEED , isAtTop() ? 0 : MAX_UP_SPEED);
        metric("Lifter/rawSpeed", desiredSpeed);
        metric("Lifter/speed", speed);
        _lifterSpark.set(speed);
    }

    public void setWheelieSpeed(double desiredSpeed) {
        double speed = limit(desiredSpeed, -1, 1);
        metric("Wheelie/rawSpeed", desiredSpeed);
        metric("Wheelie/speed", speed);
        _wheelieVictor.set(speed);
    }

    public void enableBrakeMode() {
        if (_lifterSpark ==null) { return; }
        _lifterSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_lifterSpark ==null) { return; }
        _lifterSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public double getRawNeoEncoder() {
        if (_lifterSpark ==null) { return 0; }
        return _neoStiltEncoder.getPosition();
    }

    public double getPosition() {
        return getRawNeoEncoder() - _offset;
    }

    public boolean isAtTop() {
        return _topHall.get();// || (Math.abs(getPosition()-TOP_POSITION)<TOLERANCE);
    }

    public boolean isAtBottom() {
        return _bottomHall.get();// || (Math.abs(getPosition()-BOTTOM_POSITION)<TOLERANCE);
    }

    public boolean isAtMiddle() {
        return _middleHall.get() || getPosition()>=MIDDLE_POSITION;
    }

    public boolean isOnSurface() {
        return _downIR.getAverageValue() > 1000;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveStilt(this, _robot.getOI()));
    }
    @Override
    public void updateDashboard() {
        metric("NEOEncoder", getRawNeoEncoder());
        metric("Position", getPosition());
        metric("AtTop", isAtTop());
        metric("AtBottom", isAtBottom());
        metric("AtMiddle", isAtMiddle());
        metric("IRValue", _downIR.getAverageValue());
        metric("IRVoltage", _downIR.getAverageVoltage());
        metric("OnSurface", isOnSurface());
    }
}