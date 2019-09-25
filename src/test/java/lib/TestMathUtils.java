package lib;

import com.gemsrobotics.lib.utils.MathUtils;
import static com.gemsrobotics.lib.utils.MathUtils.lerp;
import org.junit.Test;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

public class TestMathUtils {
    @Test
    public void testBounds() {
        final var a = new MathUtils.Bounds(0.0, 1.0);

        assertThat(a.constrain(0.0), is(0.0));
        assertThat(a.constrain(0.5), is(0.5));
        assertThat(a.constrain(1.0), is(1.0));

        assertThat(a.constrain(2.0), is(1.0));
        assertThat(a.constrain(-1.0), is(0.0));
    }

    @Test
    public void testLerp() {
        assertThat(lerp(0, 10, 0), is(0.0));
        assertThat(lerp(0, 10, 1), is(10.0));

        assertThat(lerp(0, 10, 1.5), is(10.0));
        assertThat(lerp(0, 10, -1.0), is(0.0));

        assertThat(lerp(0, 10, 0.75), is(7.5));
        assertThat(lerp(0, 15, 0.2), is(3.0));
        assertThat(lerp(67, -12, 0.6), is(19.6));

        assertThat(lerp(Double.POSITIVE_INFINITY, 0, 0.01), is(Double.POSITIVE_INFINITY));
        assertThat(lerp(-16.0e30, 16.0, 1.0), is(16.0));
        assertThat(lerp(1.0e20, 1.0, 1.0), is(1.0));
    }
}
