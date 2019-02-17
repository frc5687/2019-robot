package org.frc5687.deepspace.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.subsystems.Gripper;
import org.frc5687.deepspace.robot.subsystems.Spear;

/**
 * Runs the vaccuum motor and finishes (without stopping the motor) once the cargo is secured.
 */
public class GripCargo extends OutliersCommand {
    public Gripper _gripper;
    public Spear _spear;
    public OI _oi;

    public GripCargo(Gripper gripper, OI oi, Spear spear){
        _gripper = gripper;
        _spear = spear;
        _oi = oi;
        requires(_gripper);
        requires(_spear);
    }

    @Override
    protected void initialize() {
        _spear.open();
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
