package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class CargoIntake extends CommandGroup {

    public CargoIntake(Robot robot) {
        addSequential(new WristDown(robot, robot.getWrist()));
        addSequential(new StartRoller(robot.getRoller(), true));
        addSequential(new StopRoller(robot.getRoller()));
        addSequential(new WristUp(robot, robot.getWrist()));
    }
}