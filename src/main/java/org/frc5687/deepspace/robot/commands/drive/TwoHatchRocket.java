package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.AutoAlign;
import org.frc5687.deepspace.robot.commands.AutoDrive;
import org.frc5687.deepspace.robot.commands.MoveElevatorToSetPoint;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class TwoHatchRocket extends CommandGroup {
    public TwoHatchRocket(Robot robot) {
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), 30, 1,2000,2,"Align To Rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .5, false, 0));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(),-12,1,true, true, 2,"reverse 12 inches", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-175,1,2000,2,"Aligning to Station"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .5, false, 0));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(),-12,1,true, true, 2,"reverse 12 inches", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),0,1,2000,2,"Aligning to Station"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .5, true, 20));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.Ramp, robot.getOI(), 0.0));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(),20,.4,true, true, 2,"drive until shock triggered", 2000));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, robot.getOI(), 0.0));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(),-12,1,true, true, 2,"reverse 12 inches", 2000));
        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-175,1,2000,2,"Aligning to Station"));
    }
}
