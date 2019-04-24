package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.AutoAlign;
import org.frc5687.deepspace.robot.commands.AutoDrive;
import org.frc5687.deepspace.robot.commands.AutoLaunch;

public class TwoHatchCargoRocket extends CommandGroup {
    public TwoHatchCargoRocket(Robot robot, boolean OffHAB, boolean left) {
        if (OffHAB) {
            addSequential(new AutoLaunch(robot));
        }
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, 1,1000, 2,"Aligning for CargoSide"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), 30, 1, true, true, 0, "Moveing to the Side Cargo Ship", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? 90 : -90, 1,1000, 2,"Aligning to CargoSide"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 1, false, 0));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -6, 1, true, true, 0, "Moveing to the Side Cargo Ship", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, 1,1000, 2,"Aligning to CargoSide"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 1, false, 0));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, 1, true, true, 0, "Moveing to the Side Cargo Ship", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, 1,1000, 2,"Aligning to CargoSide"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -150, 1, true, true, 0, "Moving to align with back rocket", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -165 : 165, 1,1000, 2,"Aligning to CargoSide"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 1, false, 0));
    }
}
