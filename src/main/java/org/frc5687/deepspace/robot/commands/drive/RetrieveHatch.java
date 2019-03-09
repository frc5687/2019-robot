package org.frc5687.deepspace.robot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.CargoIntakeDown;
import org.frc5687.deepspace.robot.commands.intake.ClawWristDown;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class RetrieveHatch extends CommandGroup {
    public RetrieveHatch(Robot robot) {
        addSequential(new HatchMode(robot));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDriveToTarget(robot, Constants.Auto.DriveToTarget.MAX_SPEED, Constants.Auto.DriveToTarget.STOP_DISTANCE, Constants.Auto.DriveToTarget.DISTANCE_TOLERANCE, "Retrieve"));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), -12.00, Constants.Auto.DriveToTarget.MAX_SPEED, true, true, 1000, "Retreat", 1000));
    }
}