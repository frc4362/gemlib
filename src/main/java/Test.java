import com.gemsrobotics.lib.math.se2.RigidTransform;
import com.gemsrobotics.lib.math.se2.Rotation;
import com.gemsrobotics.lib.math.se2.Translation;
import com.gemsrobotics.lib.utils.Units;

public class Test {
    public static void main(String[] args) {
        
		final var pickupPose = new RigidTransform(new Translation(25.0, -78 - (13.5 * 12)), Rotation.degrees(-55));
        System.out.println(pickupPose.transformBy(RigidTransform.fromTranslation(new Translation(0.0, -6))).toString());
    }
}
