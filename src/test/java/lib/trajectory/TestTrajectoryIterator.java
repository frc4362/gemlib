package lib.trajectory;

import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.trajectory.Trajectory;
import com.gemsrobotics.lib.trajectory.TrajectoryIterator;
import org.junit.Test;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

import java.util.Arrays;
import java.util.List;

public class TestTrajectoryIterator {
    public static final List<Translation> WAYPOINTS = Arrays.asList(
            new Translation(0.0, 0.0),
            new Translation(24.0, 0.0),
            new Translation(36.0, 12.0),
            new Translation(60.0, 12.0));

    @Test
    public void test() {
        Trajectory<Translation> trajectory = new Trajectory<>(WAYPOINTS);
        TrajectoryIterator<Translation> iterator = new TrajectoryIterator<>(trajectory.getIndexView());

        // Initial conditions.
        assertThat(0.0, closeTo(iterator.getProgress(), Epsilon));
        assertThat(3.0, closeTo(iterator.getRemainingProgress(), Epsilon));
        assertThat(WAYPOINTS.get(0), equalTo(iterator.getState()));
        assertThat(iterator.isDone(), is(false));

        // Advance forward.
        assertThat(WAYPOINTS.get(0).interpolate(WAYPOINTS.get(1), 0.5), equalTo(iterator.preview(0.5).getState()));
        assertThat(WAYPOINTS.get(0).interpolate(WAYPOINTS.get(1), 0.5), equalTo(iterator.advance(0.5).getState()));
        assertThat(0.5, closeTo(iterator.getProgress(), Epsilon));
        assertThat(2.5, closeTo(iterator.getRemainingProgress(), Epsilon));
        assertThat(iterator.isDone(), is(false));

        // Advance backwards.
        assertThat(WAYPOINTS.get(0).interpolate(WAYPOINTS.get(1), 0.25), equalTo(iterator.preview(-0.25).getState()));
        assertThat(WAYPOINTS.get(0).interpolate(WAYPOINTS.get(1), 0.25), equalTo(iterator.advance(-0.25).getState()));
        assertThat(0.25, closeTo(iterator.getProgress(), Epsilon));
        assertThat(2.75, closeTo(iterator.getRemainingProgress(), Epsilon));
        assertThat(iterator.isDone(), is(false));

        // Advance past end.
        assertThat(WAYPOINTS.get(3), equalTo(iterator.preview(5.0).getState()));
        assertThat(WAYPOINTS.get(3), equalTo(iterator.advance(5.0).getState()));
        assertThat(3.0, closeTo(iterator.getProgress(), Epsilon));
        assertThat(0.0, closeTo(iterator.getRemainingProgress(), Epsilon));
        assertThat(iterator.isDone(), is(true));

        // Advance past beginning.
        assertThat(WAYPOINTS.get(0), equalTo(iterator.preview(-5.0).getState()));
        assertThat(WAYPOINTS.get(0), equalTo(iterator.advance(-5.0).getState()));
        assertThat(0.0, closeTo(iterator.getProgress(), Epsilon));
        assertThat(3.0, closeTo(iterator.getRemainingProgress(), Epsilon));
        assertThat(iterator.isDone(), is(false));
    }
}
