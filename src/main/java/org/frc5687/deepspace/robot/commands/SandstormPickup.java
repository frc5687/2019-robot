package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeUp;
import org.frc5687.deepspace.robot.commands.intake.ClawWristUp;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class SandstormPickup extends CommandGroup {
    public SandstormPickup(Robot robot) {
        addSequential(new GripClaw(robot.getHatchIntake(), Constants.Intake.CLOSE_CLAW_MILLI_SS));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.ClearRoller, Elevator.MotionMode.Ramp, null));
        addSequential(new ClawWristUp(robot));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null));
        addSequential(new SetConfiguration(robot, Robot.Configuration.hatch));
    }

}
 