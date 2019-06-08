package org.frc5687.deepspace.robot.commands.loops;

import org.frc5687.deepspace.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LimelightProcessor implements Loop {
    NetworkTable _table;
    RobotState _robotState;
    NetworkTableEntry _ledmode;
    NetworkTableEntry _cammode;
    NetworkTableEntry _pipeline;
    NetworkTableEntry _stream;

    public List<NetworkTableEntry> _target1, _target2, _combinedTarget;
    public NetworkTableEntry  _cornerX, _cornerY;

    boolean _updatesAllowed = true;


    public LimelightProcessor() {
    }

    @Override
    public void onStart(double timestamp) {
        _table = NetworkTableInstance.getDefault().getTable("limelight");
        _ledmode = _table.getEntry("ledMode");
        _cammode = _table.getEntry("camMode");
        _pipeline = _table.getEntry("pipeline");
        _stream = _table.getEntry("stream");
        _target1 = Arrays.asList(_table.getEntry("tx0"), _table.getEntry("ty0"), _table.getEntry("ta0"));
        _target2 = Arrays.asList(_table.getEntry("tx1"), _table.getEntry("ty1"), _table.getEntry("ta1"));
        _combinedTarget = Arrays.asList(_table.getEntry("tx"), _table.getEntry("ty"), _table.getEntry("ta"), _table.getEntry("tv"));
        _cornerX = _table.getEntry("tcornx");
        _cornerY = _table.getEntry("tcorny");
        setPipeline(Pipeline.TapeTrackingClosest);
    }

    @Override
    public void onLoop(double timestamp) {
        List<TargetInfo> targets = new ArrayList<>();
        if (seesTarget() && _updatesAllowed) {
            targets.add(getTargetInfo(_target1));
            targets.add(getTargetInfo(_target2));
            targets.add(new TargetInfo(Math.tan(Math.toRadians(_combinedTarget.get(0).getDouble(0))), Math.tan(Math.toRadians(_combinedTarget.get(1).getDouble(0)))));
        }

        _robotState.addVisionUpdate(timestamp, targets);
    }
    @Override
    public void onStop (double timestamp) {
    }

    public List<TargetInfo> getTargetInfos() {
        List<Double> leftXs = new ArrayList<>();
        for (int i = 0; i < 4; i++ ) {
            leftXs.add(_cornerX.getDoubleArray(new double[8]) [i]);
        }
        double firstAverageX = 0.0;
        for (double x : leftXs) {
            firstAverageX += x;
        }
        firstAverageX /= 4.0;

        List<Double> rightXs = new ArrayList<>();
        for (int i = 4; i < 8; i++) {
            rightXs.add(_cornerX.getDoubleArray(new double[8]) [i]);
        }
        double secondAverageX = 0.0;
        for (double x : rightXs) {
            secondAverageX += x;
        }
        secondAverageX /= 4.0;

        List<Double> leftYs = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            leftYs.add(_cornerY.getDoubleArray(new double[8]) [i]);
        }
        double firstAverageY = 0.0;
        for (double y : leftYs) {
            firstAverageY += y;
        }
        firstAverageY /= 4.0;

        List<Double> rightYs = new ArrayList<>();
        for (int i = 4; i < 8; i++) {
            rightYs.add(_cornerY.getDoubleArray(new double[8]) [i]);
        }

        double secondAverageY = 0.0;
        for (double y : rightYs) {
            secondAverageY += y;
        }
        secondAverageY /= 4.0;

        List<TargetInfo> targets = new ArrayList<>();
        targets.add(getTargetInfo((1.0/160.0) * (firstAverageX - 159.5), (1.0/120.0) * (119.5 - firstAverageY)));
        targets.add(getTargetInfo((1.0/160.0) * (secondAverageX - 159.5), (1.0/120.0) * (119.5 - secondAverageY)));

        return targets;
    }

    public TargetInfo getTargetInfo(double nx, double ny) {
        //vision processor width
        double vpw = 2.0 * Math.tan(Math.toRadians(Constants.Limelight.CAMERA_X_FOV));
        //vision processor height
        double vph = 2.0 * Math.tan(Math.toRadians(Constants.Limelight.CAMERA_Y_FOV));
        double x = vpw / 2.0 * nx;
        double y = vph / 2.0 * ny;
        double ax = Math.atan2(x, 1.0);
        double ay = Math.atan2(y, 1.0);

        return new TargetInfo(Math.tan(ax), Math.tan(ay));
    }

    public TargetInfo getTargetInfo(List<NetworkTableEntry> target) {
        double nx = target.get(0).getDouble(0);
        double ny = target.get(1).getDouble(0);
        double vpw = 2.0 * Math.tan(Math.toRadians(Constants.Limelight.CAMERA_X_FOV));
        double vph = 2.0 * Math.tan(Math.toRadians(Constants.Limelight.CAMERA_Y_FOV));
        double x = vpw / 2.0 * nx;
        double y = vph / 2.0 * ny;
        double ax = Math.atan2(x, 1.0);
        double ay = Math.atan2(y, 1.0);

        return new TargetInfo(Math.tan(ax), Math.tan(ay));
    }



    private boolean seesTarget(){
        boolean targetInSight = (_combinedTarget.get(3).getDouble(0) == 1.0) ? true : false;
        return targetInSight;
    }

    public void enableUpdates(boolean enable){
        _updatesAllowed = enable;
    }

    public void setPipeline(Pipeline pipeline) {
        _pipeline.setNumber(pipeline.getValue());
    }

    public void setPipeline(int pipeline) {
        _pipeline.setNumber(pipeline);
    }

    public enum Pipeline {
        TapeTrackingLargest(0),
        TapeTrackingClosest(1),
        TapeTrackingHighest(2),
        CargoTrackingLargest(8),
        CargoTrackingClosest(9);

        private int _value;

        Pipeline(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
