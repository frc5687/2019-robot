package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;

public class AutoScoreDoubleRocket extends CommandGroup {
    public AutoScoreDoubleRocket(Robot robot, boolean left) {
        String start = left ? "LeftL2" : "RightL2";
        String rocket = left ? "LeftRocket" : "RightRocket";
        String loading = left ? "LeftLoadingStation" : "RightLoadingStation";
        String center = left ? "LeftCenterField" : "RightCenterField";
        String farRocket = left ? "FarLeftRocket" : "FarRightRocket";

        addSequential(new SandstormPickup(robot));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), start +"To" + rocket, 80, false));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(),-12, .6, false, true, 1000, "Retreat", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -179.9 : 179.9, 1.0, 1000, 1.0, "Align Home"));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), rocket +"To" + loading, 80, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), loading +"To" + center, 0, true));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), center +"To" + farRocket, 80, false));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(),robot.getElevator(), -12, .6, false, true, 1000, "Retrieve", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? 180 : -180, 1.0, 1000, 1.0, "Align Cargo Ship"));
        addSequential(new CargoMode(robot));
    }
}
