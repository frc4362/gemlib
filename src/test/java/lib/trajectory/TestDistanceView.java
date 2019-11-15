package lib.trajectory;

import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.trajectory.DistanceView;
import com.gemsrobotics.lib.trajectory.Trajectory;
import org.junit.Test;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

import java.util.Arrays;
import java.util.List;

public class TestDistanceView {
    @Test
    public void test() {
        // Specify desired waypoints.
        List<Translation> waypoints = Arrays.asList(
                new Translation(0.0, 0.0),
                new Translation(24.0, 0.0),
                new Translation(36.0, 0.0),
                new Translation(36.0, 24.0),
                new Translation(60.0, 24.0));

        // Create the reference trajectory (straight line motion between waypoints).
        final Trajectory<Translation> trajectory = new Trajectory<>(waypoints);
        final DistanceView<Translation> distanceView = new DistanceView<>(trajectory);

        assertThat(0.0, closeTo(distanceView.getFirstInterpolant(), Epsilon));
        assertThat(84.0, closeTo(distanceView.getLastInterpolant(), Epsilon));

        assertThat(waypoints.get(0), is(distanceView.sample(0.0).getState()));
        assertThat(waypoints.get(0).interpolate(waypoints.get(1), 0.5), is(distanceView.sample(12.0).getState()));
        assertThat(waypoints.get(3).interpolate(waypoints.get(4), 0.5), is(distanceView.sample(72.0).getState()));
    }
}
