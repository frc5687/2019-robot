package org.frc5687.deepspace.robot.commands.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.Constants;
import org.frc5687.deepspace.robot.utils.CrashTrackingRunnable;

import java.util.ArrayList;
import java.util.List;

public class Looper {
    public final double period_k = Constants.LOOPER_DT;

    private boolean _running;

    private final Notifier _notifier;
    private final List<Loop> _loops;
    private final Object _taskRunningLock = new Object();
    private double _timestamp = 0;
    private double _dt = 0;

    private final CrashTrackingRunnable _runnable = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (_taskRunningLock) {
                if (_running) {
                    double now = Timer.getFPGATimestamp();

                    for (Loop loop : _loops) {
                        loop.onLoop(now);
                    }

                    _dt = now - _timestamp;
                    _timestamp = now;
                }
            }
        }
    };

    public Looper() {
        _notifier = new Notifier(_runnable);
        _loops = new ArrayList<>();
        _running = false;
    }

    public synchronized void register(Loop loop) {
        synchronized (_taskRunningLock) {
            _loops.add(loop);
        }
    }

    public synchronized void start() {
        if (!_running) {
            System.out.println("Starting Loops");
            synchronized (_taskRunningLock) {
                _timestamp = Timer.getFPGATimestamp();
                for (Loop loop : _loops) {
                    loop.onStart(_timestamp);
                }
                _running = true;
            }
            _notifier.startPeriodic(period_k);
        }
    }

    public synchronized void stop() {
        if (_running) {
            System.out.println("Stopping Loops");
            _notifier.stop();
            synchronized (_taskRunningLock) {
                _running = false;
                _timestamp = Timer.getFPGATimestamp();
                for (Loop loop : _loops) {
                    System.out.println("Stopping " + loop);
                    loop.onStop(_timestamp);
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt", _dt);
    }





}
