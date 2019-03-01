package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.ClawWristUp;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeUp;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class HatchMode extends CommandGroup {
    public HatchMode(Robot robot) {
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
        addSequential(new ClawWristUp(robot));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null));
        addSequential(new SetConfiguration(robot, Robot.Configuration.hatch));

    }

}
