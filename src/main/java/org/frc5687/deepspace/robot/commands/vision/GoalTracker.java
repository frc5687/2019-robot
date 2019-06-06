package org.frc5687.deepspace.robot.commands.vision;

import org.frc5687.deepspace.robot.utils.math.Translation2D;

import javax.sound.midi.Track;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import static org.frc5687.deepspace.robot.Constants.Vision.*;

/**
 * This is used in the event that multiple goals are detected to judge all goals based on timestamp, stability, and
 * continuation of previous goals (i.e. if a goal was detected earlier and has changed locations). This allows the robot
 * to make consistent decisions about which goal to aim at and to smooth out jitter from vibration of the camera.
 */
public class GoalTracker {
    /**
     * Track reports contain all of the relevant information about a given goal track.
     */
    public static class TrackReport {
        // Translation from the field frame to the goal
        public Translation2D fieldToGoal;

        // The timestamp of the latest time that the goal has been observed
        public double latestTimestamp;

        // The percentage of the goal tracking time during which this goal has
        // been observed (0 to 1)
        public double stability;

        // The track ID
        public int ID;

        public TrackReport(GoalTrack track) {
            this.fieldToGoal = track.getSmoothedPosition();
            this.latestTimestamp = track.getLatestTimestamp();
            this.stability = track.getStability();
            this.ID = track.getID();
        }
    }

    /**
     * TrackReportComparators are used in the case that multiple tracks are active (e.g. we see or have recently seen
     * multiple goals). They contain heuristics used to pick which track we should aim at by calculating a score for
     * each track (highest score wins).
     */
    public static class TrackReportComparator implements Comparator<TrackReport> {
        double stabilityWeight;
        double ageWeight;
        double currentTimestamp;
        double switchingWeight;
        int lastTrackID;

        public TrackReportComparator(double stabilityWeight, double ageWeight, double switchingWeight, int lastTrackID, double currentTimestamp) {
            this.stabilityWeight = stabilityWeight;
            this.ageWeight = ageWeight;
            this.switchingWeight = switchingWeight;
            this.lastTrackID = lastTrackID;
            this.currentTimestamp = currentTimestamp;
        }

        double score(TrackReport report) {
            double stabilityScore = report.stability * stabilityWeight;
            double ageScore = ageWeight * Math.max(0,(MAX_GOAL_TRACK_AGE - (currentTimestamp - report.latestTimestamp)) / MAX_GOAL_TRACK_AGE);
            double switchingScore = (report.ID == lastTrackID ? switchingWeight : 0);
            return stabilityScore + ageScore + switchingScore;

        }
        @Override
        public int compare(TrackReport o1, TrackReport o2) {
            double diff = score(o1) - score(o2);
            // Greater that 0 if o1 is better that o2
            if (diff < 0) {
                return 1;
            } else if (diff > 0) {
                return -1;
            } else {
                return 0;
            }
        }

    }

    List<GoalTrack> currentTracks = new ArrayList<>();
    int nextId = 0;

    public GoalTracker() {
    }

    public void rest() {
        currentTracks.clear();
    }

    public void update(double timestamp, List<Translation2D> fieldToGoals) {
        // try to update tracks
        for (Translation2D target : fieldToGoals) {
            boolean hasUpdateTrack = false;
            for (GoalTrack track : currentTracks) {
                if (!hasUpdateTrack) {
                    if (track.tryUpdate(timestamp, target)) {
                        hasUpdateTrack = true;
                    }
                } else {
                    track.emptyUpdate();
                }
            }
        }
        // prune any tracks that have died
        for (Iterator<GoalTrack> it = currentTracks.iterator(); it.hasNext();) {
            GoalTrack track = it.next();
            if (!track.isAlive()) {
                it.remove();
            }
        }
        // if all tracks are dead, start new tracks for any detections
        if (currentTracks.isEmpty()) {
            for (Translation2D target : fieldToGoals) {
                currentTracks.add(GoalTrack.makeNewTrack(timestamp, target, nextId));
                ++nextId;
            }
        }
    }

    public boolean hasTracks() {
        return !currentTracks.isEmpty();
    }

    public List<TrackReport> getTracks() {
        List<TrackReport> rv = new ArrayList<>();
        for (GoalTrack track : currentTracks) {
            rv.add(new TrackReport(track));
        }
        return rv;
    }



}
