import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.PoseBuffer;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

public class PoseBufferTest {
  @Test
  void test() {
    PoseBuffer poseBuffer = new PoseBuffer();
    ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
    for (int i = 0; i < 30; i++) {
      Pose2d p =
          new Pose2d(Math.random() * 100, Math.random() * 100, new Rotation2d(Math.random()));
      poseBuffer.pushToBuffer(p, i);
      poses.add(p);
    }
    for (double i = 10; i < 20; i++) {
      Pose2d interp1 = poses.get((int) i).interpolate(poses.get((int) (i + 1)), 0.5);
      Pose2d interp2 = poseBuffer.getPoseAtTimestamp(i + 0.5).get();
      assert Math.abs(interp1.getX() - interp2.getX()) < 0.00001;
      assert Math.abs(interp1.getY() - interp2.getY()) < 0.00001;
      assert Math.abs(interp1.getRotation().getDegrees() - interp2.getRotation().getDegrees())
          < 0.00001;
    }
  }
}
