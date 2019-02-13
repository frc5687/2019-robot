package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class CargoMode extends CommandGroup {
    public CargoMode(Robot robot) {
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.PID));
        addSequential(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Intake, Arm.HallEffectSensor.INTAKE, Arm.MotionMode.Simple));
        addSequential(new WristUp(robot, robot.getWrist()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.PID));
    }
}
