package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveElevator;
import org.frc5687.deepspace.robot.utils.HallEffect;
import org.frc5687.deepspace.robot.utils.Helpers;
import org.frc5687.deepspace.robot.utils.LaserRangeFinder;

import java.io.IOException;



public class Elevator extends OutliersSubsystem{

    private CANSparkMax _elevator;
    private Encoder _elevatorEncoder;
    private CANEncoder _neoElevatorEncoder;
    private Robot _robot;
    private LaserRangeFinder _rangeFinder;

    private HallEffect _topHall;
    private HallEffect _bottomHall;
    private int stopVariable = 0;


    public Elevator(Robot robot) {
        _robot = robot;

        _elevator = new CANSparkMax(RobotMap.CAN.SPARKMAX.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        _elevatorEncoder = new Encoder(RobotMap.DIO.ELEVATOR_A, RobotMap.DIO.ELEVATOR_B);
        _neoElevatorEncoder = _elevator.getEncoder();

        _topHall = new HallEffect(RobotMap.DIO.ELEVATOR_TOP_HALL);
        _bottomHall = new HallEffect(RobotMap.DIO.ELEVATOR_BOTTOM_HALL);
        _elevator.setInverted(Constants.Elevator.ELEVATOR_MOTOR_ELEVATOR);
        _rangeFinder = new LaserRangeFinder(LaserRangeFinder._rangeFinder_I2CADDR,0x1);

    }
    public void setElevatorSpeeds(double speed) {
        speed = Helpers.limit(speed, Constants.Elevator.MAX_ELEVATOR_SPEED);
        if (speed > 0 && isAtTop()) {
            speed = 0;
        } else if (speed < 0 && isAtBottom()) {
            speed = 0;
        }
        metric("ElevatorSpeed",speed);

        _elevator.set(speed);
    }


    public double getRawNeoEncoder() {
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
        metric("LaserRange", getRange());

    }
    public boolean isAtTop() { return _topHall.get(); }

    public boolean isAtBottom() { return _bottomHall.get(); }

    public double getRange() { return _rangeFinder.range(); }
}
