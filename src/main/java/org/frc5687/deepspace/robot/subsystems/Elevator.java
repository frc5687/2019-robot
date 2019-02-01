package org.frc5687.deepspace.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.RobotMap;
import org.frc5687.deepspace.robot.commands.DriveElevator;
import org.frc5687.deepspace.robot.utils.Helpers;

public class Elevator extends Subsystem {

    private CANSparkMax _elevator;
    private Encoder _elevatorEncoder;
    private Robot _robot;

    public Elevator(Robot robot) {
        _robot = robot;

        _elevator = new CANSparkMax(RobotMap.CAN.SPARKMAX.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        _elevatorEncoder = new Encoder(RobotMap.DIO.ELEVATOR_A, RobotMap.DIO.ELEVATOR_B);

    }
    public void setSpeeds(double speed) {
        speed = Helpers.limit(speed, Constants.Elevator.MAX_ELEVATOR_SPEED);

        DriverStation.reportError("Elevator at" + speed, false);

        _elevator.set(speed);
    }






    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveElevator(_robot, this));
    }
}
