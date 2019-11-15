package lib;

import com.gemsrobotics.lib.math.interpolation.InterpolatingDouble;
import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.utils.MathUtils;

import org.junit.Test;

import javax.annotation.processing.SupportedAnnotationTypes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static com.gemsrobotics.lib.utils.MathUtils.*;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

public class TestMathUtils {
    @Test
    public void testBounds() {
        final var a = new MathUtils.Bounds(0.0, 1.0);

        assertThat(a.coerce(0.0), is(0.0));
        assertThat(a.coerce(0.5), is(0.5));
        assertThat(a.coerce(1.0), is(1.0));

        assertThat(a.coerce(2.0), is(1.0));
        assertThat(a.coerce(-1.0), is(0.0));
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

    // we really only need to test double since its up to the implementation of the Interpolatable if the others work
    @Test
    public void testInterpolatingAverage() {
        final var a = List.of(0.0).stream().map(InterpolatingDouble::new).collect(Collectors.toList());
        assertThat(average(a).value, closeTo(0.0, Epsilon));

        final var rng = new Random();
        for (int i = 0; i < 100; i++) {
            final var ns = IntStream.range(0, 100).mapToDouble(__ -> rng.nextDouble() * 100.0).toArray();
            final var correctAverage = DoubleStream.of(ns).average().getAsDouble();
            assertThat(average(Arrays.stream(ns).mapToObj(InterpolatingDouble::new).collect(Collectors.toList())).value, closeTo(correctAverage, Epsilon));
        }
    }

    @Test
    public void testInverseSqrt() {
        final var rng = new Random();

        for (int i = 0; i < 100; i++) {
            final var n = rng.nextDouble() * rng.nextInt(100);
            assertThat(1 / Math.sqrt(n), closeTo(MathUtils.inverseSqrt(n), Epsilon));
        }
    }
}
