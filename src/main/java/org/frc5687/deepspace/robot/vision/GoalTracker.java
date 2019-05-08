package org.frc5687.deepspace.robot.vision;

import org.frc5687.deepspace.robot.Constants;
import team254.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

public class GoalTracker {

    public static class TrackReport {

        public Translation2d fieldToGoal;

        public double latestTimestamp;

        public double stability;

        public int id;

        public TrackReport(GoalTrack track) {
            this.fieldToGoal = track.mSmoothPosition;
            this.latestTimestamp = track.getLatestTimestamp();
            this.stability = track.getStability();
            this.id = track.getId();
        }
    }

    public static class TrackReportComparator implements Comparator<TrackReport> {

        double mStabilityWeight;
        double mAgeWeight;
        double mCurrentTimestamp;
        double mSwitichingWeight;
        int mLastTrackId;

        public TrackReportComparator(double stabilityWeight, double ageWeight, double switchingWeight, int lastTrackId, double currentTimestamp) {
            this.mStabilityWeight = stabilityWeight;
            this.mAgeWeight = ageWeight;
            this.mSwitichingWeight = switchingWeight;
            this.mLastTrackId = lastTrackId;
            this.mCurrentTimestamp = currentTimestamp;
        }

        double score(TrackReport report) {
            double stabilityScore = mStabilityWeight * report.stability;
            double ageScore = mAgeWeight * Math.max(0,(Constants.Vision.MAX_GOAL_TRACK_AGE - (mCurrentTimestamp - report.latestTimestamp)) / Constants.Vision.MAX_GOAL_TRACK_AGE);
            double switchingScore = (report.id == mLastTrackId ? mSwitichingWeight : 0);
            return stabilityScore + ageScore + switchingScore;
        }
        @Override
        public int compare(TrackReport o1, TrackReport o2) {
            double diff = score(o1) - score(o2);
            if (diff < 0) {
                return 1 ;
            } else if (diff > 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }
    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    int mNextId;

    public GoalTracker() {}

    public void reset() {
        mCurrentTracks.clear();
    }
    double xTarget = 0.0;
    double acceptableError = Constants.Vision.ACCEPTABLE_ERROR;
    boolean useXTarget = false;

    public void setXTarget(double x, double error) {
        xTarget = x;
        acceptableError = error;
        useXTarget = true;
    }
    public void enableXTarget(boolean enable) {
        useXTarget = enable;
    }

    public void update(double timestamp, List<Translation2d> fieldToGoal) {
        /**
         * 254's Code from 2016
         */
//        boolean hasUpdateTrack = false;
//        for (Translation2d target : fieldToGoal) {
//            for (GoalTrack track : mCurrentTracks) {
//                if (!hasUpdateTrack) {
//                    if (track.tryUpdate(timestamp, target)) {
//                        hasUpdateTrack = true;
//                    }
//                } else {
//                    track.emptyUpdate();
//                }
//            }
//        }
        /**
         * 1323's Code from 2019
         */
        if (fieldToGoal.size() >= 3 && mCurrentTracks.size() >= 3) {
            if ((Math.abs(fieldToGoal.get(2).x() - xTarget) <= acceptableError) || !useXTarget){
                for(int i = 0; i < 3; i++) {
                    mCurrentTracks.get(i).forceUpdate(timestamp, fieldToGoal.get(i));
                }
            }
        } else {
            for (GoalTrack track : mCurrentTracks) {
                track.emptyUpdate();
            }
        }

        for (Iterator<GoalTrack> it = mCurrentTracks.iterator(); it.hasNext();) {
            GoalTrack track = it.next();
            if (!track.isAlive()) {
                it.remove();
            }
        }
        if (mCurrentTracks.isEmpty()) {
            for (Translation2d target : fieldToGoal) {
                mCurrentTracks.add(GoalTrack.makeNewTrack(timestamp, target, mNextId));
                ++mNextId;
            }
        }
    }

    public boolean hasTracks() {
        return !mCurrentTracks.isEmpty();
    }

    public List<TrackReport> getTracks() {
        //"rv" = "return value"
        List<TrackReport> rv = new ArrayList<>();
        for (GoalTrack track : mCurrentTracks) {
            rv.add(new TrackReport(track));
        }
        return rv;
    }

    public synchronized void clearTracks() {
        mCurrentTracks = new ArrayList<>();
    }

}
