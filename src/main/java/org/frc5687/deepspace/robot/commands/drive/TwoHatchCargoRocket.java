package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.AutoAlign;
import org.frc5687.deepspace.robot.commands.AutoDrive;
import org.frc5687.deepspace.robot.commands.AutoLaunch;
import org.frc5687.deepspace.robot.commands.SandstormPickup;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;

public class TwoHatchCargoRocket extends CommandGroup {
    public TwoHatchCargoRocket(Robot robot, boolean OffHAB, boolean left) {
        if (OffHAB) {
            addSequential(new AutoLaunch(robot));
        }
        addParallel(new SandstormPickup(robot));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -1, .6, false,true, 0, "reverse 12 inches", 200));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -90 : 90, .8, 500, 2, "aligning to HP Station"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), 50, .7, false,true, 0, "reverse 12 inches", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, .8, 500, 2, "aligning to HP Station"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -25, .7, false,true, 0, "reverse 12 inches", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -20 : 20, 1, 1000, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .7, false,true, 0, "reverse 12 inches", 1000));


    }
}
