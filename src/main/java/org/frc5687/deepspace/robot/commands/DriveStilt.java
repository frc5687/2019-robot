package org.frc5687.deepspace.robot.commands;

import org.frc5687.deepspace.robot.OI;
import org.frc5687.deepspace.robot.Robot;
import org.frc5687.deepspace.robot.subsystems.Stilt;
import org.frc5687.deepspace.robot.Constants;

public class DriveStilt extends OutliersCommand {

        private Stilt _stilt;
        private OI _oi;

        public DriveStilt(Robot robot, Stilt stilt) {
            _stilt = stilt;
            _oi = robot.getOI();
            requires(_stilt);
        }
        @Override
        protected void execute() {
            double speed = _oi.getStiltSpeed();
            _stilt.drive(0);
        }

        @Override
        protected boolean isFinished() {
            return false;
        }
}
