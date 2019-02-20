package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.Arm;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.subsystems.Stilt;
import static org.frc5687.deepspace.robot.Constants.Auto.Climb.*;

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
        _arm.resetEncoder();
       _climbState = ClimbState.PositionArm;
       _driveTrain.enableBrakeMode();
    }

    @Override
    public void execute() {
        switch (_climbState) {
            case PositionArm:
                _arm.setSpeed(INITIAL_ARM_SPEED);
                if (_arm.getAngle() >= Constants.Arm.CONTACT_ANGLE) {
                    _climbState = ClimbState.MoveRollerAndStilt;
                }
                break;
            case MoveRollerAndStilt:
                _stilt.setLifterSpeed(STILT_SPEED);
                double armSpeed = Math.cos(Math.toRadians(_arm.getAngle())) * ARM_SPEED_SCALAR;
                _arm.setSpeed(armSpeed);
                if (_arm.getAngle() >= Constants.Arm.BOTTOM_ANGLE) {
                    _climbState = ClimbState.Done;
                }
                break;
            case Done:
                _stilt.setLifterSpeed(0);
                _arm.setSpeed(0);
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return _climbState==ClimbState.Done;
    }

    enum ClimbState {
        PositionArm(0),
        MoveRollerAndStilt(1),
        Done(2);

        private int _value;

        ClimbState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }
}
