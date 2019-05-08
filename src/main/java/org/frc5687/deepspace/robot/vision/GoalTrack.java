package org.frc5687.deepspace.robot.vision;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.deepspace.robot.Constants;
import team254.lib.geometry.Translation2d;

import java.util.Map;
import java.util.TreeMap;
import java.util.Iterator;

public class GoalTrack {
    // prefix m stands for "member variable"
    Map<Double, Translation2d> mObservedPositions = new TreeMap<>();
    Translation2d mSmoothPosition = null;
    int mId;

    private GoalTrack() {}
    // variable "rv" = return value
    public static GoalTrack makeNewTrack(double timestamp, Translation2d firstObservation, int id) {
        GoalTrack rv = new GoalTrack();
        rv.mObservedPositions.put(timestamp, firstObservation);
        rv.mSmoothPosition = firstObservation;
        rv.mId = id;
        return rv;
    }
    public void emptyUpdate() {
        removeByTime();
    }
     public boolean tryUpdate(double timestamp, Translation2d newObservation) {
        if (!isAlive()) {
            return false;
        }
        double distance = mSmoothPosition.inverse().translateBy(newObservation).norm();
        if (distance < Constants.Vision.MAX_TRACKER_DISTANCE) {
            mObservedPositions.put(timestamp, newObservation);
            removeByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }
     }
     public void forceUpdate(double timestamp, Translation2d newObservation) {
        if (mSmoothPosition.distance(newObservation) > Constants.Vision.MAX_TRACKER_DISTANCE) {
            mObservedPositions.clear();
        }
        mObservedPositions.put(timestamp, newObservation);
        removeByTime();
     }

    public boolean isAlive() {
        return mObservedPositions.size() > 0;
    }

    void removeByTime(){
        double deleteBefore = Timer.getFPGATimestamp() - Constants.Vision.MAX_GOAL_TRACK_AGE;
        // "it" stands for iteration.
        for (Iterator<Map.Entry<Double, Translation2d>> it = mObservedPositions.entrySet().iterator(); it.hasNext();) {
            Map.Entry<Double, Translation2d> entry = it.next();
            if (entry.getKey() < deleteBefore){
                it.remove();
            }
        }
        if (mObservedPositions.isEmpty()){
            mSmoothPosition = null;
        } else {
            smooth();
        }
    }

    void smooth(){
        if (isAlive()) {
            double x = 0;
            double y = 0;
            for (Map.Entry<Double, Translation2d> entry : mObservedPositions.entrySet()) {
                x += entry.getValue().x();
                y += entry.getValue().y();
            }
            x /= mObservedPositions.size();
            y /= mObservedPositions.size();
            mSmoothPosition = new Translation2d(x, y);
        }
    }

    public Translation2d getSmoothedPosition() {
        return mSmoothPosition;
    }

    public double getLatestTimestamp() {
        return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public double getStability() {
        return Math.min(1.0, mObservedPositions.size() / (Constants.Vision.CAMERA_FRAME_RATE * Constants.Vision.MAX_GOAL_TRACK_AGE));
    }

    public int getId(){
        return mId;
    }
}
