package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeUp;
import org.frc5687.deepspace.robot.commands.intake.ClawWristDown;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class StartingConfiguration extends CommandGroup {
    public StartingConfiguration(Robot robot) {
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.ClearRoller, Elevator.MotionMode.Ramp, null, 0.0));
        addSequential(new ClawWristDown(robot));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.Ramp, null, 0.0));
        addSequential(new SetConfiguration(robot, Robot.Configuration.starting));
    }
}
