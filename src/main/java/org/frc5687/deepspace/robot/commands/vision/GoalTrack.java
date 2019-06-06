package org.frc5687.deepspace.robot.commands.vision;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.deepspace.robot.utils.math.Translation2D;

import javax.swing.tree.TreeNode;
import java.awt.font.TransformAttribute;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import static org.frc5687.deepspace.robot.Constants.Vision.*;

/**
 * A class that is used to keep track of all goals detected by the vision system. As goals are detected/not detected
 * anymore by the vision system, function calls will be made to create, destroy, or update a goal track.
 *
 * This helps in the goal ranking process that determines which goal to fire into, and helps to smooth measurements of
 * the goal's location over time.
 */
public class GoalTrack {
    Map<Double, Translation2D> observedPositions = new TreeMap<>();
    Translation2D smoothedPosition = null;
    int ID;

    private GoalTrack() {
    }


    /**
     * Makes a new track based on the timestamp and the goal's coordinates (from vision)
     */
    public static GoalTrack makeNewTrack(double timestamp, Translation2D firstObservation, int id) {
        // rv is return value.
        GoalTrack rv = new GoalTrack();
        rv.observedPositions.put(timestamp, firstObservation);
        rv.smoothedPosition = firstObservation;
        rv.ID = id;
        return rv;
    }

    public void emptyUpdate() {
        pruneByTime();
    }

    /**
     * Attempts to update the track with a new observation.
     *
     * @return True if the track was updated
     */
    public boolean tryUpdate(double timestamp, Translation2D newObservation) {
        if (!isAlive()) {
            return false;
        }

        double distance = smoothedPosition.inverse().translateBy(newObservation).norm();
        if (distance < MAX_TRACKER_DISTANCE) {
            observedPositions.put(timestamp, newObservation);
            pruneByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }


    }

    /**
     * Removes the track if it is older than the set "age" described in the Constants file.
     */
    void pruneByTime() {
        double deleteBefore = Timer.getFPGATimestamp() - MAX_GOAL_TRACK_AGE;
        for (Iterator<Map.Entry<Double, Translation2D>> it = observedPositions.entrySet().iterator(); it.hasNext();) {
            Map.Entry<Double, Translation2D> entry = it.next();
            if (entry.getKey() < deleteBefore) {
                it.remove();
            }
        }
        if (observedPositions.isEmpty()) {
            smoothedPosition = null;
        } else {
            smooth();
        }
    }

    public boolean isAlive() {
        return observedPositions.size() > 0;
    }

    /**
     * Averages out the observed positions based on an set of observed positions
     */
    void smooth() {
        if (isAlive()) {
            double x = 0;
            double y = 0;
            for (Map.Entry<Double, Translation2D> entry : observedPositions.entrySet()) {
                x += entry.getValue().getX();
                y += entry.getValue().getY();
            }
            x /= observedPositions.size();
            y /= observedPositions.size();
            smoothedPosition = new Translation2D(x, y);
        }
    }

    public Translation2D getSmoothedPosition() {
        return smoothedPosition;
    }

    public double getLatestTimestamp() {
        return observedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public double getStability() {
        return Math.min(1.0, observedPositions.size() / (CAMERA_FRAME_RATE * MAX_GOAL_TRACK_AGE));
    }

    public int getID() {
        return ID;
    }

}
