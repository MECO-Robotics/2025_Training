package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.position_joint.PositionJoint;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Components extends SubsystemBase {
  private final PositionJoint elevator;
  private final PositionJoint pivot;

  private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d root = mech.getRoot("elevator", 1.5, 1.5);
  private final LoggedMechanismLigament2d elevatorVisualizer =
      root.append(new LoggedMechanismLigament2d("elevator", 0.5, 90));
  ;

  public Components(PositionJoint elevator, PositionJoint pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
  }

  @Override
  public void periodic() {
    elevatorVisualizer.setLength(elevator.getPosition());

    Logger.recordOutput("Components/Elevator", mech);
  }
}
