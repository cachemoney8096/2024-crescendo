package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

public class PoseBuffer {
  RingBuffer<Pair<Double, Pose2d>> buffer = new RingBuffer<Pair<Double, Pose2d>>();

  public PoseBuffer() {}

  public void pushToBuffer(Pose2d pose, double timestamp) {
    Pair<Double, Pose2d> pos = new Pair<Double, Pose2d>(timestamp, pose);
    buffer.addFirst(pos);
    if (buffer.size() > 50) {
      buffer.removeLast();
    }
  }

  public int size() {
    return buffer.size();
  }

  public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
    if (buffer.getFromFirst(0).getFirst() <= timestamp
        && buffer.getFromLast(0).getFirst() >= timestamp) {
      return Optional.empty();
    }
    if (buffer.size() < 2) {
      return Optional.empty();
    }
    for (int i = 0; i < buffer.size() - 1; i++) {
      if (buffer.getFromFirst(i).getFirst() >= timestamp
          && buffer.getFromFirst(i + 1).getFirst() <= timestamp) {
        Pair<Double, Pose2d> timedPoseA = buffer.getFromFirst(i);
        Pair<Double, Pose2d> timedPoseB = buffer.getFromFirst(i + 1);
        double percentage =
            (timedPoseA.getFirst() - timestamp) / (timedPoseA.getFirst() - timedPoseB.getFirst());
        return Optional.of(timedPoseA.getSecond().interpolate(timedPoseB.getSecond(), percentage));
      }
    }
    return Optional.empty();
  }
}
