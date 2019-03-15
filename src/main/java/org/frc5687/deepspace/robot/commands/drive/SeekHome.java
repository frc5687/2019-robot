package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.AutoAlign;
import org.frc5687.deepspace.robot.commands.AutoDrive;

public class SeekHome extends CommandGroup {
    public SeekHome(Robot robot) {
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -12, .6, false, true, 1000, "", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), 180.0, 1.0, 1000, 1.0, "Align Home"));
    }
}
