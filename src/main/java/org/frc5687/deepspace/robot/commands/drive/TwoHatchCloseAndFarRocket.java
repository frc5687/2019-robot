package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;

public class TwoHatchCloseAndFarRocket extends CommandGroup {
    public TwoHatchCloseAndFarRocket (Robot robot, boolean OffHAB, boolean left) {
        if (OffHAB) {
            addSequential(new AutoLaunch(robot));
        }
        addParallel(new SandstormPickup(robot));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, .7, 500, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, true));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -8, .7, false, true, 0, "reverse 12 inches", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -175 : 175, .7, 1000, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -179 : 179, .7, 250, 5, "aligning to rocket"));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), left ? "LeftFarRocket" : "RightFarRocket", 0, true));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, false, 3));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -4, .7, false, true, 0, "reverse 12 inches", 200));
        addParallel(new CargoMode(robot));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? 90 : -90, .7, 1000, 5, "aligning to rocket"));
        addSequential(new AutoIntake(robot));
    }

}
