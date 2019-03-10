package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.*;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class IntakeCargo extends CommandGroup {

    public IntakeCargo(Robot robot) {
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null, 0.0));
        addSequential(new StartRoller(robot.getCargoIntake(), true));
        addSequential(new StopRoller(robot.getCargoIntake()));
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
    }
}
