package org.frc5687.deepspace.robot.utils;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;

public class CANToF {
    private static final int kPWF = 8;
    private static final int kPWFToF = 10;

    private CAN _can;
    private Notifier _notifier;

    public CANToF(int id) {
        _can = new CAN(id, kPWF, kPWFToF);

        _notifier = new Notifier(this::process);
        _notifier.startPeriodic(.05);
    }

    protected void process() {
        // Try to read a packet
        _can.readPacketNew()
    }

}
