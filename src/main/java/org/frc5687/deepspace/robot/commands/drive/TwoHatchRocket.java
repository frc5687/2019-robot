package org.frc5687.deepspace.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.*;
import org.frc5687.deepspace.robot.commands.intake.GripClaw;
import org.frc5687.deepspace.robot.commands.intake.PointClaw;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class TwoHatchRocket extends CommandGroup {
//    public TwoHatchRocket(Robot robot, boolean left) {
//        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, 1,750,2,"Align To Rocket"));
//        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 1, false, 0));
//        addSequential(new PointClaw(robot.getHatchIntake()));
//        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(),robot.getElevator(),-7,.4,false, true, 0,"reverse 12 inches", 1000));
//        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-160,.5,1000,2,"Aligning to Station"));
//        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 1, false, 0));
//        addSequential(new GripClaw(robot.getHatchIntake()));
//        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12,.7,false, true, 0,"reverse 12 inches", 2000));
//        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-160,.5,1000,2,"Aligning to Station"));
//        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -150,1,false, true, 0,"reverse 12 inches", 2000));
////        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-180,.5,500,2,"Aligning to Station"));
//        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -30,.7,false, true, 0,"reverse 12 inches", 1000));
//        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-150,.5,1000,2,"Aligning to Station"));
////        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),0,1,2000,2,"Aligning to Station"));
//        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 1, false, 0));
//        addSequential(new PointClaw(robot.getHatchIntake()));
//        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -7,.7,false, true, 0,"reverse 12 inches", 2000));
//        addSequential(new AutoAlign(robot.getDriveTrain(),robot.getIMU(),-175,1,2000,2,"Aligning to Station"));
//    }
    public TwoHatchRocket(Robot robot, boolean OffHAB, boolean left) {
        if (OffHAB) {
            addSequential(new AutoLaunch(robot));
        }
        addParallel(new SandstormPickup(robot));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, .7, 500, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, true));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -10, .7, false,true, 0, "reverse 12 inches", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, .7, 1000, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .7,false, true, 0, "reverse 12 inches", 500));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, .7, 200, 2, "aligning to rocket"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -72, .7,false, true, 0, "reverse 12 inches", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, .7, 1000, 2, "aligning to rocket"));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch3, Elevator.MotionMode.Ramp, robot.getOI(), 0.0));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .6, false, 0, false));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .4,false, true, 0, "reverse 12 inches", 1000));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, robot.getOI(), 0.0));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, .7, 1000, 2, "aligning to rocket"));



    }
}
