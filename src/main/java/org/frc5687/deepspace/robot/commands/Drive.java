package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.PDP;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;

    private boolean _autoAlignEnabled = false;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        requires(_driveTrain);

        logMetrics("StickSpeed", "StickRotation", "LeftPower", "RightPower", "LeftMasterAmps", "LeftFollowerAmps", "RightMasterAmps", "RightFollowerAmps");
    }



    @Override
    protected void initialize() {
        // create the _angleController here, just like in AutoDriveToTarget
    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();

        // If the auto-align trigger is pressed, and !_autoAlignEnabled:
        //   Enable the LEDs
        // else if auto_align trigger is not pressed, and _autoAlignEnabled
        //   disable the LEDs, disable the controller
        // else if _autoAlignEnabled
        //   Get target info (copy from AutoAlignToTarget)
        //   If target sighted and ither controller not enabled or new setpoint different enough from old setpoint
        //      set setPoint
        //      enable controller



        // If autoAlignEnabled and pidControllerEnabled, send pidOut in place of wheelRotation (you may need a scale override flag as discussed earlier)
        _driveTrain.cheesyDrive(stickSpeed, wheelRotation, _oi.isCreepPressed());

        metric("StickSpeed", stickSpeed);
        metric("StickRotation", wheelRotation);
        metric("LeftPower", _driveTrain.getLeftPower());
        metric("RightPower", _driveTrain.getRightPower());
        metric("LeftMasterAmps", _driveTrain.getLeftMasterCurrent());
        metric("LeftFollowerAmps",_driveTrain.getLeftFollowerCurrent());
        metric("RightMasterAmps",_driveTrain.getRightMasterCurrent());
        metric("RightFollowerAmps",_driveTrain.getRightFollowerCurrent());
    }



    @Override
    protected boolean isFinished() {
        return false;
    }
}
