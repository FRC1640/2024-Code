package frc.lib.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.SwerveDriveDimensions;

public class SparkMaxOdometryThread {
    private final Notifier notifier;

    private static SparkMaxOdometryThread instance = null;
    private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
    private List<Queue<Double>> queues = new ArrayList<>();
    private List<Queue<Double>> timestampQueues = new ArrayList<>();

    public static SparkMaxOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkMaxOdometryThread();
        }
        return instance;
    }

    private SparkMaxOdometryThread() {
        notifier = new Notifier(this::periodic);
        notifier.setName("SparkMaxOdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / SwerveDriveDimensions.odometryFrequency);
        }
    }

    public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        DriveSubsystem.odometryLock.lock();
        try {
            signals.add(signal);
            queues.add(queue);
        } finally {
            DriveSubsystem.odometryLock.unlock();
        }
        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        DriveSubsystem.odometryLock.lock();
        try {
          timestampQueues.add(queue);
        } finally {
          DriveSubsystem.odometryLock.unlock();
        }
        return queue;
      }

      private void periodic() {
        DriveSubsystem.odometryLock.lock();
        double timestamp = Logger.getRealTimestamp() / 1e6;
        try {
          double[] values = new double[signals.size()];
          boolean isValid = true;
          for (int i = 0; i < signals.size(); i++) {
            OptionalDouble value = signals.get(i).get();
            if (value.isPresent()) {
              values[i] = value.getAsDouble();
            } else {
              isValid = false;
              break;
            }
          }
          if (isValid) {
            for (int i = 0; i < queues.size(); i++) {
              queues.get(i).offer(values[i]);
            }
            for (int i = 0; i < timestampQueues.size(); i++) {
              timestampQueues.get(i).offer(timestamp);
            }
          }
        } finally {
          DriveSubsystem.odometryLock.unlock();
        }
      }
}
