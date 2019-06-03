package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;

public class AutoScoreFrontCargo extends CommandGroup {
    public AutoScoreFrontCargo(Robot robot, int starting, boolean left) {
        String start = starting < 0 ? "LeftL2" : starting > 0 ? "RightL2" : "Center";
        String mid = left ? "LeftCargoFace" : "RightCargoFace";
        String end = left ? "LeftLoadingStation" : "RightLoadingStation";

        addSequential(new SandstormPickup(robot));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), start +"To" + mid, 80, false));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(),-12, .6, false, true, 1000, "Retreat", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -135 : 235, 1.0, 1000, 1.0, "Align Home"));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), mid + "To" + end, 80, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(),-12, .6, false, true, 1000, "Retrieve", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), 0, 1.0, 1000, 1.0, "Align Rocket"));
    }
}
