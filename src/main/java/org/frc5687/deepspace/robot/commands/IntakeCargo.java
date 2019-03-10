package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.commands.intake.*;
import org.frc5687.deepspace.robot.subsystems.Elevator;

public class IntakeCargo extends CommandGroup {

    public IntakeCargo(Robot robot) {
//        addSequential(new GripClaw(robot.getIntake()));
//        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.Ramp));
        addSequential(new ClawWristDown(robot));
        addSequential(new CargoIntakeDown(robot.getCargoIntake()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null, 0.0));
        addSequential(new StartRoller(robot.getCargoIntake(), true));
        addSequential(new StopRoller(robot.getCargoIntake()));
        // addSequential(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Secure, Arm.HallEffectSensor.SECURE, Arm.MotionMode.Simple));
      //  addSequential(new StartGripper(robot.getGripper()));
        //addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Secure, Elevator.MotionMode.PID));
      //  addSequential(new GripCargo(robot.getGripper(), robot.getOI()));
        //addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.PID));
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
        //addSequential(new MoveArmToSetPoint(robot.getArm(), Arm.Setpoint.Stowed, Arm.HallEffectSensor.STOWED, Arm.MotionMode.Simple));
    }
}
