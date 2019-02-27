package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.subsystems.*;

import static org.frc5687.deepspace.robot.Constants.Auto.Climb.*;
import static org.frc5687.deepspace.robot.Constants.Arm.*;

public class AutoClimb extends OutliersCommand {
    private Stilt _stilt;
    private Arm _arm;
    private DriveTrain _driveTrain;

    private CargoIntake _cargoIntake;
    private HatchIntake _hatchIntake;

    private ClimbState _climbState;
    private double _angleCos;
    private double _encoderOffset;
    private boolean isDone = false;
    private long _stiltTimeout = 0;

    public AutoClimb(Stilt stilt, Arm arm, DriveTrain driveTrain, CargoIntake cargoIntake, HatchIntake hatchIntake) {
        _stilt = stilt;
        _arm = arm;
        _driveTrain = driveTrain;

        _cargoIntake = cargoIntake;
        _hatchIntake = hatchIntake;

        requires(_stilt);
        requires(_arm);
        requires(_driveTrain);
    }

    @Override
    protected void initialize() {
       _climbState =  ClimbState.StowArm;
       _driveTrain.enableBrakeMode();
        metric("ClimbState", _climbState.name());
    }

    @Override
    public void execute() {
        switch (_climbState) {
            case StowArm:
                _cargoIntake.raiseWrist();
                _hatchIntake.lowerWrist();
                _arm.enableBrakeMode();
                _stilt.enableBrakeMode();
                _arm.setSpeed(STOW_SPEED);
                if (_arm.isLeftStowed() && _arm.isRightStowed()) {
                    DriverStation.reportError("Transitioning to " + ClimbState.PositionArm.name(), false);
                    _climbState = ClimbState.PositionArm;
                }
                break;
            case PositionArm:
                _arm.setSpeed(INITIAL_ARM_SPEED);
                if (_arm.getAngle() >= CONTACT_ANGLE) {
                    DriverStation.reportError("Transitioning to " + ClimbState.MoveRollerAndStilt.name(), false);
                    _climbState = ClimbState.MoveRollerAndStilt;
                }
                break;
            case MoveRollerAndStilt:
                _stilt.setLifterSpeed(STILT_SPEED);
                metric("StiltSpeed", STILT_SPEED);
                double armSpeed =  _arm.getAngle() >= SLOW_ANGLE ? ARM_SLOW_SPEED : ARM_SPEED; // Math.cos(Math.toRadians(_arm.getAngle())) * ARM_SPEED_SCALAR;

                _arm.setSpeed(armSpeed);
                metric("ArmSpeed", armSpeed);
                if ((_arm.isLow() || _arm.getAngle() >= BOTTOM_ANGLE)
                && _stilt.isAtTop()) {
                    metric("StiltSpeed", 0);
                    metric("ArmSpeed", 0);
                    _arm.setSpeed(ARM_HOLD_SPEED);
                    _driveTrain.disableBrakeMode();
                    DriverStation.reportError("Transitioning to " + ClimbState.WheelieForward.name(), false);
                    _climbState = ClimbState.WheelieForward;
                }
                break;
            case WheelieForward:
                _stilt.setLifterSpeed(STILT_HOLD_SPEED);
                _stilt.setWheelieSpeed(WHEELIE_FORWARD_SPEED);
                _driveTrain.cheesyDrive(DRIVE_FORWARD_SPEED,0);
                metric("WheelieSpeed", WHEELIE_FORWARD_SPEED);
                metric("DriveSpeed", DRIVE_FORWARD_SPEED);
                metric("StiltSpeed", STILT_HOLD_SPEED);
                if (_stilt.isOnSurface()) {
                    metric("WheelieSpeed", 0);
                    metric("DriveSpeed", 0);
                    metric("StiltSpeed", 0);
                    _arm.enableCoastMode();
                    _stilt.setWheelieSpeed(0);
                    DriverStation.reportError("Transitioning to " + ClimbState.LiftStilt.name(), false);
                    _climbState = ClimbState.LiftArm;
                }
                break;
            case LiftArm:
                _arm.setSpeed(RAISE_ARM_SPEED);
                if (_arm.getAngle() <= ARM_RETRACT_ANGLE) {
                    _arm.setSpeed(0);
                    _climbState = ClimbState.LiftStilt;
                    _stiltTimeout = System.currentTimeMillis() + STILT_TIMEOUT;
                }
                break;

            case LiftStilt:
                _stilt.setLifterSpeed(RAISE_STILT_SPEED);
                metric("StiltSpeed", RAISE_STILT_SPEED);
                if (_stilt.isAtBottom()) {
                    _stilt.enableCoastMode();
                    _driveTrain.resetDriveEncoders();
                    _climbState = ClimbState.Park;
                }
                if (System.currentTimeMillis() >= _stiltTimeout) {
                    _stilt.setLifterSpeed(0);
                    _stilt.enableCoastMode();
                    _climbState = ClimbState.WaitStilt;
                }
                break;
            case WaitStilt:
                _stilt.setLifterSpeed(0);
                metric("StiltSpeed", 0);
                if (_stilt.isAtBottom()) {
                    _stilt.enableCoastMode();
                    _driveTrain.resetDriveEncoders();
                    _climbState = ClimbState.Park;
                }
                break;

            case Park:
                metric("DriveSpeed", PARK_SPEED);
                _driveTrain.cheesyDrive(PARK_SPEED, 0);
                if (_driveTrain.getDistance() > PARK_DISTANCE) {
                    metric("DriveSpeed", 0);
                    _driveTrain.cheesyDrive(0.0,0);
                    DriverStation.reportError("Transitioning to " + ClimbState.Done.name(), false);
                    _climbState = ClimbState.Done;
                }
                break;
            case Done:
                _stilt.setLifterSpeed(0);
                _arm.setSpeed(0);
                break;
        }
        metric("ClimbState", _climbState.name());
    }

    @Override
    protected void end() {
        // _stilt.enableCoastMode();
        // _arm.enableCoastMode();
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
        LiftArm(4),
        LiftStilt(5),
        WaitStilt(6),
        Park(7),
        Done(8);

        private int _value;

        ClimbState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }
}
