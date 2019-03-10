package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.*;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class IntakeHatch extends CommandGroup {
     public IntakeHatch(Robot robot) {
         addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null));
         addSequential(new PointClaw(robot.getHatchIntake()));
         addSequential(new CargoIntakeUp(robot.getCargoIntake()));
         addSequential(new ClawWristUp(robot));
     }
}
