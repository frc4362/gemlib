package lib;

import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;

import static com.gemsrobotics.lib.utils.MathUtils.epsilonEquals;
import static com.gemsrobotics.lib.utils.MathUtils.kEpsilon;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.*;

import com.gemsrobotics.lib.math.se2.Twist;
import org.junit.Test;

public class TestSE2Math {
	@Test
	public void testRotation2d() {
		// Test constructors
		final var rot1 = new Rotation();
		assertThat(rot1.cos(), closeTo(1.0, kEpsilon));
		assertThat(rot1.sin(), closeTo(0.0, kEpsilon));
		assertThat(rot1.tan(), closeTo(0.0, kEpsilon));
		assertThat(rot1.getDegrees(), closeTo(0.0, kEpsilon));
		assertThat(rot1.getRadians(), closeTo(0.0, kEpsilon));

		final var rot2 = new Rotation(1, 1, true);
        assertThat(Math.sqrt(2) / 2, closeTo(rot2.cos(), kEpsilon));
        assertThat(Math.sqrt(2) / 2, closeTo(rot2.sin(), kEpsilon));
        assertThat(1.0, closeTo(rot2.tan(), kEpsilon));
        assertThat(45.0, closeTo(rot2.getDegrees(), kEpsilon));
        assertThat(Math.PI / 4, closeTo(rot2.getRadians(), kEpsilon));

		final var rot3 = Rotation.radians(Math.PI / 2);
		assertThat(0.0, closeTo(rot3.cos(), kEpsilon));
		assertThat(1.0, closeTo(rot3.sin(), kEpsilon));
		assertThat(1 / kEpsilon, lessThan(rot3.tan()));
		assertThat(90.0, closeTo(rot3.getDegrees(), kEpsilon));
		assertThat(Math.PI / 2, closeTo(rot3.getRadians(), kEpsilon));

		final var rot4 = Rotation.degrees(270);
		assertThat(0.0, closeTo(rot4.cos(), kEpsilon));
		assertThat(-1.0, closeTo(rot4.sin(), kEpsilon));
		assertThat(-1 / kEpsilon, greaterThan(rot4.tan()));
		assertThat(-90.0, closeTo(rot4.getDegrees(), kEpsilon));
		assertThat(-Math.PI / 2, closeTo(rot4.getRadians(), kEpsilon));

		// Test inversion
		final var rot5 = rot4.inverse();
		assertThat(0.0, closeTo(rot5.cos(), kEpsilon));
		assertThat(1.0, closeTo(rot5.sin(), kEpsilon));
		assertThat(1.0 / kEpsilon, lessThan(rot5.tan()));
		assertThat(90.0, closeTo(rot5.getDegrees(), kEpsilon));
		assertThat(Math.PI / 2, closeTo(rot5.getRadians(), kEpsilon));

		final var rot6 = Rotation.degrees(1);
		final var rot7 = rot6.inverse();
		assertThat(rot6.cos(), closeTo(rot7.cos(), kEpsilon));
		assertThat(-rot6.sin(), closeTo(rot7.sin(), kEpsilon));
		assertThat(-1.0, closeTo(rot7.getDegrees(), kEpsilon));

		// Test rotateBy
		final var rot8 = Rotation.degrees(45);
		final var rot9 = Rotation.degrees(45);
		final var rot10 = rot8.rotateBy(rot9);
        assertThat(0.0, closeTo(rot10.cos(), kEpsilon));
        assertThat(1.0, closeTo(rot10.sin(), kEpsilon));
        assertThat(1 / kEpsilon, lessThan(rot10.tan()));
		assertThat(90.0, closeTo(rot10.getDegrees(), kEpsilon));
		assertThat(Math.PI / 2, closeTo(rot10.getRadians(), kEpsilon));

		final var rot11 = Rotation.degrees(45);
		final var rot12 = Rotation.degrees(-45);
		final var rot13 = rot11.rotateBy(rot12);
		assertThat(1.0, closeTo(rot13.cos(), kEpsilon));
		assertThat(0.0, closeTo(rot13.sin(), kEpsilon));
		assertThat(0.0, closeTo(rot13.tan(), kEpsilon));
		assertThat(0.0, closeTo(rot13.getDegrees(), kEpsilon));
		assertThat(0.0, closeTo(rot13.getRadians(), kEpsilon));

		// A rotation times its inverse should be the identity
		final var identity = new Rotation();
		final var rot14 = Rotation.degrees(21.45);
		final var rot15 = rot14.rotateBy(rot14.inverse());
		assertThat(identity.cos(), closeTo(rot15.cos(), kEpsilon));
		assertThat(identity.sin(), closeTo(rot15.sin(), kEpsilon));
		assertThat(identity.getDegrees(), closeTo(rot15.getDegrees(), kEpsilon));

		// Test interpolation
		final var rot16 = Rotation.degrees(45);
		final var rot17 = Rotation.degrees(135);
		final var rot18 = rot16.interpolate(rot17, .5);
		assertThat(90.0, closeTo(rot18.getDegrees(), kEpsilon));

		final var rot19 = Rotation.degrees(45);
		final var rot20 = Rotation.degrees(135);
		final var rot21 = rot19.interpolate(rot20, .75);
		assertThat(112.5, closeTo(rot21.getDegrees(), kEpsilon));

		final var rot22 = Rotation.degrees(45);
		final var rot23 = Rotation.degrees(-45);
		final var rot24 = rot22.interpolate(rot23, .5);
		assertThat(0.0, closeTo(rot24.getDegrees(), kEpsilon));

		final var rot25 = Rotation.degrees(45);
		final var rot26 = Rotation.degrees(45);
		final var rot27 = rot25.interpolate(rot26, .5);
		assertThat(45.0, closeTo(rot27.getDegrees(), kEpsilon));

		final var rot28 = Rotation.degrees(45);
		final var rot29 = Rotation.degrees(45);
		final var rot30 = rot28.interpolate(rot29, .5);
		assertThat(45.0, closeTo(rot30.getDegrees(), kEpsilon));

		// Test parallel.
		final var rot31 = Rotation.degrees(45);
		final var rot32 = Rotation.degrees(45);
		assertThat(rot31.isParallel(rot32), is(true));

		final var rot33 = Rotation.degrees(45);
		final var rot34 = Rotation.degrees(-45);
		assertThat(rot33.isParallel(rot34), is(false));

		final var rot35 = Rotation.degrees(45);
		final var rot36 = Rotation.degrees(-135);
		assertThat(rot35.isParallel(rot36), is(true));
	}

    @Test
    public void testTranslation2d() {
        // Test constructors
        final var pos1 = new Translation();
        assertThat(0.0, closeTo(pos1.x(), kEpsilon));
        assertThat(0.0, closeTo(pos1.y(), kEpsilon));
        assertThat(0.0, closeTo(pos1.norm(), kEpsilon));

        final var pos2 = new Translation(3, 4);
        assertThat(3.0, closeTo(pos2.x(), kEpsilon));
        assertThat(4.0, closeTo(pos2.y(), kEpsilon));
        assertThat(5.0, closeTo(pos2.norm(), kEpsilon));

        // Test inversion
        final var pos3 = new Translation(3.152, 4.1666);
        final var pos4 = pos3.inverse();
        assertThat(-pos3.x(), closeTo(pos4.x(), kEpsilon));
        assertThat(-pos3.y(), closeTo(pos4.y(), kEpsilon));
        assertThat(pos3.norm(), closeTo(pos4.norm(), kEpsilon));

        // Test rotateBy
        final var pos5 = new Translation(2, 0);
        final var rot1 = Rotation.degrees(90);
        final var pos6 = pos5.rotateBy(rot1);
        assertThat(0.0, closeTo(pos6.x(), kEpsilon));
        assertThat(2.0, closeTo(pos6.y(), kEpsilon));
        assertThat(pos5.norm(), closeTo(pos6.norm(), kEpsilon));

        final var pos7 = new Translation(2, 0);
        final var rot2 = Rotation.degrees(-45);
        final var pos8 = pos7.rotateBy(rot2);
        assertThat(Math.sqrt(2), closeTo(pos8.x(), kEpsilon));
        assertThat(-Math.sqrt(2), closeTo(pos8.y(), kEpsilon));
        assertThat(pos7.norm(), closeTo(pos8.norm(), kEpsilon));

        // Test translateBy
        final var pos9 = new Translation(2, 0);
        final var pos10 = new Translation(-2, 1);
        final var pos11 = pos9.translateBy(pos10);
        assertThat(0.0, closeTo(pos11.x(), kEpsilon));
        assertThat(1.0, closeTo(pos11.y(), kEpsilon));
        assertThat(1.0, closeTo(pos11.norm(), kEpsilon));

        // A translation times its inverse should be the identity
        final var identity = new Translation();
        final var pos12 = new Translation(2.16612, -23.55);
        final var pos13 = pos12.translateBy(pos12.inverse());
        assertThat(identity.x(), closeTo(pos13.x(), kEpsilon));
        assertThat(identity.y(), closeTo(pos13.y(), kEpsilon));
        assertThat(identity.norm(), closeTo(pos13.norm(), kEpsilon));

        // Test interpolation
        final var pos14 = new Translation(0, 1);
        final var pos15 = new Translation(10, -1);
        final var pos16 = pos14.interpolate(pos15, .5);
        assertThat(5.0, closeTo(pos16.x(), kEpsilon));
        assertThat(0.0, closeTo(pos16.y(), kEpsilon));

        final var pos17 = new Translation(0, 1);
        final var pos18 = new Translation(10, -1);
        final var pos19 = pos17.interpolate(pos18, .75);
        assertThat(7.5, closeTo(pos19.x(), kEpsilon));
        assertThat(-.5, closeTo(pos19.y(), kEpsilon));
    }

	@Test
	public void testTwist() {
		// Exponentiation (integrate twist to obtain a RigidTransform2d)
		final var twist1 = new Twist(1.0, 0.0, 0.0);
		final var rt1 = RigidTransform.ofTwist(twist1);
		assertThat(1.0, closeTo(rt1.getTranslation().x(), kEpsilon));
		assertThat(0.0, closeTo(rt1.getTranslation().y(), kEpsilon));
		assertThat(0.0, closeTo(rt1.getRotation().getDegrees(), kEpsilon));

		// Scaled.
		final var twist2 = new Twist(1.0, 0.0, 0.0);
		final var rt2 = RigidTransform.ofTwist(twist2.scaled(2.5));
		assertThat(2.5, closeTo(rt2.getTranslation().x(), kEpsilon));
		assertThat(0.0, closeTo(rt2.getTranslation().y(), kEpsilon));
		assertThat(0.0, closeTo(rt2.getRotation().getDegrees(), kEpsilon));

		// Logarithm (find the twist to apply to obtain a given RigidTransform2d)
		final var rt3 = new RigidTransform(new Translation(2.0, 2.0), Rotation.radians(Math.PI / 2));
		final var twist3 = RigidTransform.toTwist(rt3);
		assertThat(Math.PI, closeTo(twist3.dx, kEpsilon));
		assertThat(0.0, closeTo(twist3.dy, kEpsilon));
		assertThat(Math.PI / 2, closeTo(twist3.dtheta, kEpsilon));

		// Logarithm is the inverse of exponentiation.
		final var rt4 = RigidTransform.ofTwist(twist3);
		assertThat(rt4.getTranslation().x(), closeTo(rt3.getTranslation().x(), kEpsilon));
		assertThat(rt4.getTranslation().y(), closeTo(rt3.getTranslation().y(), kEpsilon));
		assertThat(rt4.getRotation().getDegrees(), closeTo(rt3.getRotation().getDegrees(), kEpsilon));
	}

    @Test
    public void testRigidTransform2d() {
        // Test constructors
        final var pose1 = new RigidTransform();
        assertThat(0.0, closeTo(pose1.getTranslation().x(), kEpsilon));
        assertThat(0.0, closeTo(pose1.getTranslation().y(), kEpsilon));
        assertThat(0.0, closeTo(pose1.getRotation().getDegrees(), kEpsilon));

        final var pose2 = new RigidTransform(new Translation(3, 4), Rotation.degrees(45));
        assertThat(3.0, closeTo(pose2.getTranslation().x(), kEpsilon));
        assertThat(4.0, closeTo(pose2.getTranslation().y(), kEpsilon));
        assertThat(45.0, closeTo(pose2.getRotation().getDegrees(), kEpsilon));

        // Test transformation
        final var pose3 = new RigidTransform(new Translation(3, 4), Rotation.degrees(90));
        final var pose4 = new RigidTransform(new Translation(1, 0), Rotation.degrees(0));
        final var pose5 = pose3.transformBy(pose4);
        assertThat(3.0, closeTo(pose5.getTranslation().x(), kEpsilon));
        assertThat(5.0, closeTo(pose5.getTranslation().y(), kEpsilon));
        assertThat(90.0, closeTo(pose5.getRotation().getDegrees(), kEpsilon));

        final var pose6 = new RigidTransform(new Translation(3, 4), Rotation.degrees(90));
        final var pose7 = new RigidTransform(new Translation(1, 0), Rotation.degrees(-90));
        final var pose8 = pose6.transformBy(pose7);
        assertThat(3.0, closeTo(pose8.getTranslation().x(), kEpsilon));
        assertThat(5.0, closeTo(pose8.getTranslation().y(), kEpsilon));
        assertThat(0.0, closeTo(pose8.getRotation().getDegrees(), kEpsilon));

        // A pose times its inverse should be the identity
        final var identity = new RigidTransform();
        final var pose9 = new RigidTransform(new Translation(3.51512152, 4.23), Rotation.degrees(91.6));
        final var pose10 = pose9.transformBy(pose9.inverse());
        assertThat(identity.getTranslation().x(), closeTo(pose10.getTranslation().x(), kEpsilon));
        assertThat(identity.getTranslation().y(), closeTo(pose10.getTranslation().y(), kEpsilon));
        assertThat(identity.getRotation().getDegrees(), closeTo(pose10.getRotation().getDegrees(), kEpsilon));

        // Test interpolation
        // Movement from pose1 to pose2 is along a circle with radius of 10 units centered at (3, -6)
        final var pose11 = new RigidTransform(new Translation(3, 4), Rotation.degrees(90));
        final var pose12 = new RigidTransform(new Translation(13, -6), Rotation.degrees(0.0));
        final var pose13 = pose11.interpolate(pose12, .5);
        final var expectedAngle = Math.PI / 4;
        assertThat(3.0 + 10.0 * cos(expectedAngle), closeTo(pose13.getTranslation().x(), kEpsilon));
        assertThat(-6.0 + 10.0 * sin(expectedAngle), closeTo(pose13.getTranslation().y(), kEpsilon));
        assertThat(expectedAngle, closeTo(pose13.getRotation().getRadians(), kEpsilon));

        final var pose14 = new RigidTransform(new Translation(3, 4), Rotation.degrees(90));
        final var pose15 = new RigidTransform(new Translation(13, -6), Rotation.degrees(0.0));
        final var pose16 = pose14.interpolate(pose15, .75);
        final var expectedAngle2 = Math.PI / 8.0;
        assertThat(3.0 + 10.0 * cos(expectedAngle2), closeTo(pose16.getTranslation().x(), kEpsilon));
        assertThat(-6.0 + 10.0 * sin(expectedAngle2), closeTo(pose16.getTranslation().y(), kEpsilon));
        assertThat(expectedAngle2, closeTo(pose16.getRotation().getRadians(), kEpsilon));
    }
}
