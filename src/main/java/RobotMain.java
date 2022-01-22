import com.gemsrobotics.demo2022.Demobot;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotMain {
	public static void main(final String[] args) {
		RobotBase.startRobot(Demobot::new);
	}
}
