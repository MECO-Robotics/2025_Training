package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionJointConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    // Not supported by TalonFX
    SINE
  }

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_CANCODER_PRO,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  public record PositionJointGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance,
      double kDefaultSetpoint) {}

  // Position Joint Gear Ratio should be multiplied by Math.PI * 2 for rotation joints to convert
  // from rotations to radians
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointGains ELEVATOR_GAINS_SIM =
      new PositionJointGains(10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10, 11},
          new boolean[] {false},
          8,
          40,
          GravityType.CONSTANT,
          EncoderType.EXTERNAL_CANCODER,
          11,
          Rotation2d.fromRotations(0.0),
          "");

  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointGains PIVOT_GAINS_SIM =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointHardwareConfig PIVOT_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {12},
          new boolean[] {false},
          4,
          40,
          GravityType.COSINE,
          EncoderType.EXTERNAL_CANCODER,
          11,
          Rotation2d.fromRotations(0.0),
          "");
}
