package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PositionJoint extends SubsystemBase {
  private final PositionJointIO positionJoint;
  private final PositionJointIOInputsAutoLogged inputs = new PositionJointIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kG;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber kMaxVelo;
  private final LoggedTunableNumber kMaxAccel;

  private final LoggedTunableNumber kMinPosition;
  private final LoggedTunableNumber kMaxPosition;

  private final LoggedTunableNumber kTolerance;

  private final LoggedTunableNumber kSetpoint;

  private TrapezoidProfile.Constraints constraints;

  private TrapezoidProfile profile;

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public PositionJoint(PositionJointIO io, PositionJointGains gains) {
    super(io.getName());

    positionJoint = io;
    name = positionJoint.getName();

    kP = new LoggedTunableNumber(name + "/Gains/Feedback/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/Feedback/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/Feedback/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/Feedforward/kS", gains.kS());
    kG = new LoggedTunableNumber(name + "/Gains/Feedforward/kG", gains.kG());
    kV = new LoggedTunableNumber(name + "/Gains/Feedforward/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/Feedforward/kA", gains.kA());

    kMaxVelo = new LoggedTunableNumber(name + "/Gains/Constraints/kMaxVelo", gains.kMaxVelo());
    kMaxAccel = new LoggedTunableNumber(name + "/Gains/Constraints/kMaxAccel", gains.kMaxAccel());

    kMinPosition =
        new LoggedTunableNumber(name + "/Gains/Constraints/kMinPosition", gains.kMinPosition());
    kMaxPosition =
        new LoggedTunableNumber(name + "/Gains/Constraints/kMaxPosition", gains.kMaxPosition());

    kTolerance = new LoggedTunableNumber(name + "/Gains/Tolerance/kTolerance", gains.kTolerance());

    kSetpoint =
        new LoggedTunableNumber(name + "/Gains/Setpoint/kSetpoint", gains.kDefaultSetpoint());

    constraints = new TrapezoidProfile.Constraints(gains.kMaxVelo(), gains.kMaxAccel());
    profile = new TrapezoidProfile(constraints);

    goal = new TrapezoidProfile.State(gains.kDefaultSetpoint(), 0);
    setpoint = goal;

    SmartDashboard.putData(name, this);
  }

  @Override
  public void periodic() {
    positionJoint.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    setpoint = profile.calculate(0.02, setpoint, goal);

    positionJoint.setPosition(setpoint.position, setpoint.velocity);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          positionJoint.setGains(
              new PositionJointGains(
                  values[0],
                  values[1],
                  values[2],
                  values[3],
                  values[4],
                  values[5],
                  values[6],
                  values[7],
                  values[8],
                  values[9],
                  values[10],
                  values[11],
                  values[12]));

          goal =
              new TrapezoidProfile.State(
                  MathUtil.clamp(values[12], kMinPosition.get(), kMaxPosition.get()), 0);

          constraints = new TrapezoidProfile.Constraints(values[7], values[8]);
          profile = new TrapezoidProfile(constraints);
        },
        kP,
        kI,
        kD,
        kS,
        kG,
        kV,
        kA,
        kMaxVelo,
        kMaxAccel,
        kMinPosition,
        kMaxPosition,
        kTolerance,
        kSetpoint);

    Logger.recordOutput(name + "/isFinished", isFinished());
  }

  public void setPosition(double position) {
    goal =
        new TrapezoidProfile.State(
            MathUtil.clamp(position, kMinPosition.get(), kMaxPosition.get()), 0);
  }

  public void incrementPosition(double deltaPosition) {
    goal.position += deltaPosition;
  }

  public void setVoltage(double voltage) {
    positionJoint.setVoltage(voltage);
  }

  public double getPosition() {
    return inputs.outputPosition;
  }

  public double getDesiredPosition() {
    return inputs.desiredPosition;
  }

  public boolean isFinished() {
    return Math.abs(inputs.outputPosition - goal.position) < kTolerance.get();
  }

  public void resetPosition() {
    positionJoint.resetPosition();
    goal.position = 0;
  }

  public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier) {
    return new PositionJointPositionCommand(positionJoint, positionSupplier);
  }

  public static Command setVelocity(PositionJoint positionJoint, DoubleSupplier velocitySupplier) {
    return new PositionJointPositionCommand(positionJoint, velocitySupplier);
  }
}
