package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.Stilt;
import static org.frc5687.deepspace.robot.Constants.Auto.Climb.*;
import static org.frc5687.deepspace.robot.Constants.Arm.*;

public class AutoClimb extends OutliersCommand {
    private Stilt _stilt;
    private Arm _arm;
    private DriveTrain _driveTrain;

    private ClimbState _climbState;
    private double _angleCos;
    private double _encoderOffset;
    private boolean isDone = false;

    public AutoClimb(Stilt stilt, Arm arm, DriveTrain driveTrain) {
        _stilt = stilt;
        _arm = arm;
        _driveTrain = driveTrain;
        requires(_stilt);
        requires(_arm);
        requires(_driveTrain);
    }

    @Override
    protected void initialize() {
       _climbState =  ClimbState.PositionArm;
       _driveTrain.enableBrakeMode();
        metric("ClimbState", _climbState.name());
    }

    @Override
    public void execute() {
        switch (_climbState) {
            case StowArm:
                _arm.setSpeed(STOW_SPEED);
                if (_arm.isLeftStowed() && _arm.isRightStowed()) {
                    _climbState = ClimbState.PositionArm;
                }
                break;
            case PositionArm:
                _arm.setSpeed(INITIAL_ARM_SPEED);
                if (_arm.getAngle() >= Constants.Arm.CONTACT_ANGLE) {
                    _climbState = ClimbState.MoveRollerAndStilt;
                }
                break;
            case MoveRollerAndStilt:
                _stilt.setLifterSpeed(STILT_SPEED);
                double armSpeed = ARM_SPEED; // Math.cos(Math.toRadians(_arm.getAngle())) * ARM_SPEED_SCALAR;
                _arm.setSpeed(armSpeed);
                if ((_arm.isLow() || _arm.getAngle() >= Constants.Arm.BOTTOM_ANGLE)
                && _stilt.isAtTop()) {
                    _climbState = ClimbState.WheelieForward;
                }
                break;
            case WheelieForward:
                _stilt.setWheelieSpeed(WHEELIE_FORWARD_SPEED);
                if (_stilt.isOnSurface()) {
                    _stilt.setWheelieSpeed(0);
                    _climbState = ClimbState.LiftStilt;
                }
            case LiftStilt:
                _stilt.setLifterSpeed(RAISE_STILT_SPEED);
                if (_stilt.isAtBottom()) {
                    _driveTrain.resetDriveEncoders();
                    _climbState = ClimbState.LiftStilt;
                }
            case Park:
                _driveTrain.cheesyDrive(PARK_SPEED, 0);
                if (_driveTrain.getDistance() > PARK_DISTANCE) {
                    _driveTrain.cheesyDrive(0.0,0);
                    _climbState = ClimbState.Done;
                }
            case Done:
                _stilt.setLifterSpeed(0);
                _arm.setSpeed(0);
                break;
        }
        metric("ClimbState", _climbState.name());
    }

    @Override
    protected boolean isFinished() {
        return _climbState==ClimbState.Done;
    }

    enum ClimbState {
        StowArm(0),
        PositionArm(1),
        MoveRollerAndStilt(2),
        WheelieForward(3),
        LiftStilt(4),
        Park(5),
        Done(6);

        private int _value;

        ClimbState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }
}
