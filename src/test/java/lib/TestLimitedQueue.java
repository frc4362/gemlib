package lib;

import com.gemsrobotics.lib.data.LimitedQueue;
import org.hamcrest.CoreMatchers;
import org.junit.Assert;
import org.junit.Test;

public class TestLimitedQueue {
    @Test
    public void testLimitedQueueCapacity() {
        final var a = new LimitedQueue<Double>(5);

        Assert.assertThat(a.hasSpaceRemaining(), CoreMatchers.is(true));

        a.add(1.0);
        a.add(2.0);
        a.add(3.0);
        a.add(4.0);
        a.add(5.0);

        Assert.assertThat(a.hasSpaceRemaining(), CoreMatchers.is(false));
    }

    @Test
    public void testLimitedQueuePurging() {
        final var a = new LimitedQueue<Integer>(3);

        a.add(1);
        a.add(2);
        a.add(3);

        Assert.assertThat(a.getLast(), CoreMatchers.is(3));
        Assert.assertThat(a.getFirst(), CoreMatchers.is(1));

        a.add(4);

        Assert.assertThat(a.getLast(), CoreMatchers.is(4));
        Assert.assertThat(a.getFirst(), CoreMatchers.is(2));
    }
}
