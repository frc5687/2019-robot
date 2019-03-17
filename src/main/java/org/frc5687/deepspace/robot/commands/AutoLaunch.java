package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;

public class AutoLaunch extends CommandGroup {
    public AutoLaunch(Robot robot) {
        addSequential(new AutoDrive(robot.getDriveTrain(),robot.getIMU(),48,.6,false, true, 0, "", 2000));
        addSequential(new HatchMode(robot));
    }
}
