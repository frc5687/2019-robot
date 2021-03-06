package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeUp;
import org.frc5687.deepspace.robot.commands.intake.ClawWristDown;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class StartingConfiguration extends CommandGroup {
    public StartingConfiguration(Robot robot) {
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.ClearBumper, Elevator.MotionMode.Simple, null, Constants.Elevator.MODE_SPEED));
        addSequential(new ClawWristDown(robot));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Simple, null, Constants.Elevator.MODE_SPEED));
        addSequential(new SetConfiguration(robot, Robot.Configuration.starting));
    }
}
