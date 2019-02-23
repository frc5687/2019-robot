package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveElevator;
import org.frc5687.deepspace.robot.utils.HallEffect;
import static org.frc5687.deepspace.robot.utils.Helpers.*;
import static org.frc5687.deepspace.robot.Constants.Elevator.*;

public class Elevator extends OutliersSubsystem implements PIDSource {

    private CANSparkMax _elevator;
    private Encoder _elevatorEncoder;
    private CANEncoder _neoElevatorEncoder;
    private Robot _robot;

    private HallEffect _topHall;
    private HallEffect _bottomHall;

    private double _offset = 0;

    public Elevator(Robot robot) {
        _robot = robot;

        try {
            _elevator = new CANSparkMax(RobotMap.CAN.SPARKMAX.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
            _elevator.setInverted(Constants.Elevator.ELEVATOR_MOTOR_INVERTED);
            _neoElevatorEncoder = _elevator.getEncoder();
        } catch (Exception e) {
            error("Unable to allocate elevator controller: " + e.getMessage());
        }
        _elevatorEncoder = new Encoder(RobotMap.DIO.ELEVATOR_B, RobotMap.DIO.ELEVATOR_A);

        _topHall = new HallEffect(RobotMap.DIO.ELEVATOR_TOP_HALL);
        _bottomHall = new HallEffect(RobotMap.DIO.ELEVATOR_BOTTOM_HALL);

    }
    public void setSpeed(double speed) {
        setSpeed(speed, false);
    }
    public void setSpeed(double speed, boolean overrideRamp) {
        speed = limit(speed, -MAX_SPEED_DOWN, MAX_SPEED_UP);
        if (!overrideRamp) {
            if (speed > 0) {
                speed = limit(speed, -MAX_SPEED_DOWN, _elevator.get() + (MAX_SPEED_UP / STEPS_UP));
            } else if (speed < 0) {
                speed = limit(speed, _elevator.get() - (MAX_SPEED_DOWN / STEPS_UP), MAX_SPEED_UP);
            }
        }
        if (isAtTop()) {
            speed = limit(speed, -MAX_SPEED_DOWN,  0);
        } else if (isNearTop()) {
            speed = limit(speed, -MAX_SPEED_DOWN,  JELLO_SPEED_UP);
        } else if (isAtBottom()) {
            speed = limit(speed, 0, MAX_SPEED_UP);
        } else if (isNearBottom()) {
            speed = limit(speed, -JELLO_SPEED_DOWN, MAX_SPEED_UP);
        }
        metric("ElevatorSpeed",speed);

        if (_elevator==null) { return; }
        if (isAtTop()) {
            resetEncoder(Setpoint.Top.getValue());
        } else if (isAtBottom()) {
            resetEncoder(Setpoint.Bottom.getValue());
        }
        _elevator.set(speed);
    }

    public void enableBrakeMode() {
        if (_elevator==null) { return; }
        _elevator.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_elevator==null) { return; }
        // _elevator.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    private double getRawNeoEncoder() {
        if (_elevator==null) { return 0; }
        return 0;
    }

    private double getRawMAGEncoder() {
        return _elevatorEncoder.get();
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveElevator(_robot, this));
    }
    @Override
    public void updateDashboard() {
        metric("MAGEncoder", getRawMAGEncoder());
        metric("Position", getPosition());
        metric("Bottom", isAtBottom());
        metric("Top", isAtTop());
    }
    public boolean isAtTop() { return _topHall.get(); }

    public boolean isNearTop() { return isAtTop() || (getPosition() > Setpoint.Top.getValue() - TOP_JELLO_ZONE); }

    public boolean isAtBottom() { return _bottomHall.get(); }

    public boolean isNearBottom() { return isAtBottom() || (getPosition() < Setpoint.Bottom.getValue() + BOTTOM_JELLO_ZONE); }

    public void resetEncoder() {
        resetEncoder(0);
    }

    public void resetEncoder(long position) {
        _offset = position - getRawMAGEncoder();
    }

    public double getPosition() {
        return getRawMAGEncoder() + _offset;
    }

    public boolean isHallEffectTriggered(HallEffectSensor hall) {
        switch(hall) {
            case TOP:
                return isAtTop();
            case BOTTOM:
                return isAtBottom();
        }
        return false;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getPosition();
    }

    public enum HallEffectSensor {
        BOTTOM,
        TOP
    }


    public enum Setpoint {
        Bottom(0, HallEffectSensor.BOTTOM),
        Port1(1, HallEffectSensor.BOTTOM),
        Hatch1(2, HallEffectSensor.BOTTOM),
        Secure(450),
        ClearRoller(935),
        HPMode(1450),
        Port2(2168),
        Hatch2(2169),
        Port3(4098, HallEffectSensor.TOP),
        Hatch3(4099, HallEffectSensor.TOP),
        Top(4100, HallEffectSensor.TOP);

        private int _value;
        private HallEffectSensor _hall;

        Setpoint(int value) {
            this._value = value;
        }

        Setpoint(int value, HallEffectSensor hall) {
            this(value);
            this._hall = hall;
        }

        public int getValue() {
            return _value;
        }

        public HallEffectSensor getHall() { return _hall; }
    }

    public enum MotionMode {
        Simple(0),
        PID(1),
        Path(2),
        Ramp(3);

        private int _value;

        MotionMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }


}
