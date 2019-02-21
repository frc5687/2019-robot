package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;

public class EjectCargo extends CommandGroup {
    public EjectCargo(Robot robot) {
        //addParallel(new StopGripper(robot.getGripper()));
        addParallel(new PointClaw(robot.getIntake()));
    }
}
