package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
    public record FlywheelGains(
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA,
            double kMaxAccel,
            double kTolerance) {
    }

    public record FlywheelHardwareConfig(
            int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {
    }

    public static final FlywheelHardwareConfig EXAMPLE_CONFIG = new FlywheelHardwareConfig(new int[] { 1 },
            new boolean[] { true }, 2.0, 40, "");

    public static final FlywheelGains EXAMPLE_GAINS = new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

    // Constants for Shooter
    public static final FlywheelHardwareConfig RIGHT_FLYWHEEL_MOTOR = new FlywheelHardwareConfig(new int[] { 21 },
            new boolean[] { false }, 1.0, 40, "rio");
    public static final FlywheelGains RIGHT_FLYWHEEL_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

    public static final FlywheelHardwareConfig LEFT_FLYWHEEL_MOTOR = new FlywheelHardwareConfig(new int[] { 22 },
            new boolean[] { false }, 1.0, 40, "rio");
    public static final FlywheelGains LEFT_FLYWHEEL_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

    // Constants for Indexer
    public static final FlywheelHardwareConfig INDEXER_MOTOR = new FlywheelHardwareConfig(new int[] { 23 },
            new boolean[] { false }, 1.0, 20, "rio");
    public static final FlywheelGains INDEXER_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

    
}
