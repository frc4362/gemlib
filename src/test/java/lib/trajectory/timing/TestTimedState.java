package lib.trajectory.timing;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.trajectory.parameterization.TimedState;
import org.junit.Test;
import static com.gemsrobotics.lib.utils.MathUtils.Epsilon;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

public class TestTimedState {
    @Test
    public void test() {
        // At (0,0,0), t=0, v=0, acceleration=1
        final TimedState<RigidTransform> startState = new TimedState<>(RigidTransform.fromTranslation(new Translation(0.0, 0.0)), 0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        final TimedState<RigidTransform> endState = new TimedState<>(RigidTransform.fromTranslation(new Translation(0.5, 0.0)), 1.0, 1.0, 0.0);

        assertThat(startState, is(startState.interpolate(endState, 0.0)));
        assertThat(endState, is(startState.interpolate(endState, 1.0)));
        assertThat(endState, is(endState.interpolate(startState, 0.0)));
        assertThat(startState, is(endState.interpolate(startState, 1.0)));

        final TimedState<RigidTransform> middleState = startState.interpolate(endState, 0.5);
        assertThat(0.5, closeTo(middleState.t(), Epsilon));
        assertThat(startState.getAcceleration(), closeTo(middleState.getAcceleration(), Epsilon));
        assertThat(0.5, closeTo(middleState.getVelocity(), Epsilon));
        assertThat(0.125, closeTo(middleState.getState().getTranslation().x(), Epsilon));
    }
}
