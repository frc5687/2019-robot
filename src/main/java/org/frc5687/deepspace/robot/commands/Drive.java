package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.DriveTrain;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        requires(_driveTrain);
    }

    @Override
    protected void execute() {
        // Get the base speed from the throttle
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();

        _driveTrain.cheesyDrive(stickSpeed, wheelRotation);
    }



    @Override
    protected boolean isFinished() {
        return false;
    }
}
