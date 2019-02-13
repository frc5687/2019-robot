package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveElevator;
import org.frc5687.deepspace.robot.utils.HallEffect;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Elevator extends OutliersSubsystem implements PIDSource {

    private CANSparkMax _elevator;
    private Encoder _elevatorEncoder;
    private CANEncoder _neoElevatorEncoder;
    private Robot _robot;

    private HallEffect _topHall;
    private HallEffect _bottomHall;

    public Elevator(Robot robot) {
        _robot = robot;

        try {
            _elevator = new CANSparkMax(RobotMap.CAN.SPARKMAX.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
            _elevator.setInverted(Constants.Elevator.ELEVATOR_MOTOR_INVERTED);
            _neoElevatorEncoder = _elevator.getEncoder();
        } catch (Exception e) {
            error("Unable to allocate elevator controller: " + e.getMessage());
        }
        _elevatorEncoder = new Encoder(RobotMap.DIO.ELEVATOR_A, RobotMap.DIO.ELEVATOR_B);

        _topHall = new HallEffect(RobotMap.DIO.ELEVATOR_TOP_HALL);
        _bottomHall = new HallEffect(RobotMap.DIO.ELEVATOR_BOTTOM_HALL);

    }
    public void setElevatorSpeeds(double speed) {
        speed = Helpers.limit(speed, -Constants.Elevator.MAX_ELEVATOR_SPEED_DOWN,  Constants.Elevator.MAX_ELEVATOR_SPEED_UP);
        if (speed > 0 && isAtTop()) {
            speed = 0;
        } else if (speed < 0 && isAtBottom()) {
            speed = 0;
        }
        metric("ElevatorSpeed",speed);

        if (_elevator==null) { return; }
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

    public double getRawNeoEncoder() {
        if (_elevator==null) { return 0; }
        return _neoElevatorEncoder.getPosition();
    }

    public double getRawMAGEncoder() {
        return _elevatorEncoder.get();
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveElevator(_robot, this));
    }
    @Override
    public void updateDashboard() {
        metric("MAGEncoder", getRawMAGEncoder());
        metric("NEOEncoder", getRawNeoEncoder());

    }
    public boolean isAtTop() { return _topHall.get(); }

    public boolean isAtBottom() { return _bottomHall.get(); }

    public double getPosition() {
        return getRawMAGEncoder();
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


    public enum Setpoint {
        Bottom(0),
        Port1(132),
        Hatch1(133),
        Secure(450),
        Port2(2168),
        Hatch2(2169),
        Port3(4100),
        Hatch3(4100),
        Top(4100);

        private int _value;

        Setpoint(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    public enum MotionMode {
        Simple(0),
        PID(1),
        Path(2);

        private int _value;

        MotionMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }


}
