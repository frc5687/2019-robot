package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Gripper;

/**
 * Runs the vaccuum motor and finishes (without stopping the motor) once the cargo is secured.
 */
public class GripCargo extends OutliersCommand {
    public Gripper _gripper;
    public OI _oi;

    public GripCargo(Gripper gripper, OI oi){
        _gripper = gripper;
        _oi = oi;
        requires(_gripper);
    }

    @Override
    protected void initialize() {
        _gripper.start();
    }

    @Override
    protected void execute(){
        _gripper.run();
    }


    @Override
    protected void end(){
    }

    @Override
    /**
     * Returns true if the gripper detects secured cargo.
     */
    protected boolean isFinished() {
        return _oi.getAbort() || _gripper.hasCargo();
    }
}
