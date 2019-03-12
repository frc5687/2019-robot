package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeDown;
import org.frc5687.deepspace.robot.commands.intake.ClawWristDown;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class CargoMode extends CommandGroup {
    public CargoMode(Robot robot) {
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.ClearRoller, Elevator.MotionMode.Ramp, null, 0.0));
        addSequential(new ClawWristDown(robot));
        addSequential(new CargoIntakeDown(robot.getCargoIntake()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null, 0.0));
        addSequential(new SetConfiguration(robot, Robot.Configuration.cargo));
    }
}
