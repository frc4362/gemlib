package lib.trajectory;

import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.trajectory.Trajectory;
import org.junit.Test;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

import java.util.Arrays;
import java.util.List;

public class TestTrajectory {
    public static final List<Translation> WAYPOINTS = Arrays.asList(
            new Translation(0.0, 0.0),
            new Translation(24.0, 0.0),
            new Translation(36.0, 12.0),
            new Translation(60.0, 12.0));

    @Test
    public void testConstruction() {
        // Empty constructor.
        Trajectory<Translation> traj = new Trajectory<>();
        assertThat(traj.isEmpty(), is(true));
        assertThat(0.0, closeTo(traj.getIndexView().getFirstInterpolant(), Epsilon));
        assertThat(0.0, closeTo(traj.getIndexView().getLastInterpolant(), Epsilon));
        assertThat(0, is(traj.length()));

        // Set states at construction time.
        traj = new Trajectory<>(WAYPOINTS);
        assertThat(traj.isEmpty(), is(false));
        assertThat(0.0, closeTo(traj.getIndexView().getFirstInterpolant(), Epsilon));
        assertThat(3.0, closeTo(traj.getIndexView().getLastInterpolant(), Epsilon));
        assertThat(4, is(traj.length()));
    }

    @Test
    public void testStateAccessors() {
        Trajectory<Translation> trajectory = new Trajectory<>(WAYPOINTS);

        assertThat(WAYPOINTS.get(0), is(trajectory.getState(0)));
        assertThat(WAYPOINTS.get(1), is(trajectory.getState(1)));
        assertThat(WAYPOINTS.get(2), is(trajectory.getState(2)));
        assertThat(WAYPOINTS.get(3), is(trajectory.getState(3)));

        assertThat(WAYPOINTS.get(0), is(trajectory.getInterpolated(0.0).getState()));
        assertThat(trajectory.getInterpolated(0.0).getIndexFloor(), is(0));
        assertThat(trajectory.getInterpolated(0.0).getIndexCeiling(), is(0));

        assertThat(WAYPOINTS.get(1), is(trajectory.getInterpolated(1.0).getState()));
        assertThat(trajectory.getInterpolated(1.0).getIndexFloor(), is(1));
        assertThat(trajectory.getInterpolated(1.0).getIndexCeiling(), is(1));

        assertThat(WAYPOINTS.get(2), is(trajectory.getInterpolated(2.0).getState()));
        assertThat(trajectory.getInterpolated(2.0).getIndexFloor(), equalTo(2));
        assertThat(trajectory.getInterpolated(2.0).getIndexCeiling(), equalTo(2));

        assertThat(WAYPOINTS.get(3), equalTo(trajectory.getInterpolated(3.0).getState()));
        assertThat(trajectory.getInterpolated(3.0).getIndexFloor(), equalTo(3));
        assertThat(trajectory.getInterpolated(3.0).getIndexCeiling(), equalTo(3));

        assertThat(WAYPOINTS.get(0).interpolate(WAYPOINTS.get(1), .25), equalTo(trajectory.getInterpolated(0.25).getState()));
        assertThat(trajectory.getInterpolated(0.25).getIndexFloor(), equalTo(0));
        assertThat(trajectory.getInterpolated(0.25).getIndexCeiling(), equalTo(1));

        assertThat(WAYPOINTS.get(1).interpolate(WAYPOINTS.get(2), .5), equalTo(trajectory.getInterpolated(1.5).getState()));
        assertThat(trajectory.getInterpolated(1.5).getIndexFloor(), equalTo(1));
        assertThat(trajectory.getInterpolated(1.5).getIndexCeiling(), equalTo(2));

        assertThat(WAYPOINTS.get(2).interpolate(WAYPOINTS.get(3), .75), equalTo(trajectory.getInterpolated(2.75).getState()));
        assertThat(trajectory.getInterpolated(2.75).getIndexFloor(), equalTo(2));
        assertThat(trajectory.getInterpolated(2.75).getIndexCeiling(), equalTo(3));

        Trajectory<Translation>.IndexView indexView = trajectory.getIndexView();
        assertThat(WAYPOINTS.get(0).interpolate(WAYPOINTS.get(1), .25), equalTo(indexView.sample(0.25).getState()));
        assertThat(WAYPOINTS.get(1).interpolate(WAYPOINTS.get(2), .5), equalTo(indexView.sample(1.5).getState()));
        assertThat(WAYPOINTS.get(2).interpolate(WAYPOINTS.get(3), .75), equalTo(indexView.sample(2.75).getState()));
    }
}
