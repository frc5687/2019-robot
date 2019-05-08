package team254.lib.geometry;

import org.frc5687.deepspace.robot.utils.Interpolable;
import team254.lib.util.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
