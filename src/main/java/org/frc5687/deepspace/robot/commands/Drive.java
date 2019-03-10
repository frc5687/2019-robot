package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;
import org.frc5687.deepspace.robot.utils.PDP;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        requires(_driveTrain);

        logMetrics("StickSpeed", "StickRotation", "LeftPower", "RightPower", "LeftMasterAmps", "LeftFollowerAmps", "RightMasterAmps", "RightFollowerAmps");
    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();

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
