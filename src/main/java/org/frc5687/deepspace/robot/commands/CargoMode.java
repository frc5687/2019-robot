package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeDown;
import org.frc5687.deepspace.robot.commands.intake.ClawWristDown;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class CargoMode extends CommandGroup {

    public CargoMode(Robot robot) {
        addParallel(new CargoIntakeDown(robot.getCargoIntake()));
        addParallel(new ClawWristDown(robot));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp));
        addSequential(new SetConfiguration(robot, Robot.Configuration.cargo));
    }
}
