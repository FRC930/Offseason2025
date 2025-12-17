
package frc.robot.subsystems.coralendeffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/**
 * <h1>Coral end effector</h1>
 * <p>Controls the rollers on the end of our coral end effector</p>
 * <ul>
 * <li>Voltage control</li>
 * </ul>
 */
public class CoralEndEffector extends SubsystemBase {

  public static final double CORAL_DISTANCE_THRESHOLD = 5.0;
  public static final double CORAL_STRENGTH_THRESHOLD = 8000.0;

  private CoralEndEffectorIO m_IO;
  private CoralEndEffectorInputsAutoLogged logged = new CoralEndEffectorInputsAutoLogged();
  private CoralEndEffectorState m_state = CoralEndEffectorState.IDLE;

  public CoralEndEffector(CoralEndEffectorIO io) {
    m_IO = io;
    logged.angularVelocity = DegreesPerSecond.mutable(0);
    logged.supplyCurrent = Amps.mutable(0);
    logged.torqueCurrent = Amps.mutable(0);
    logged.voltageSetPoint = Volts.mutable(0);
    logged.voltage = Volts.mutable(0);
    logged.coralDistance = Inches.mutable(100);
  }

  public void setTarget(Voltage target) {
    m_IO.setTarget(target);
  }

  @Override
  public void periodic() {
    m_IO.updateInputs(logged);
    Logger.processInputs("RobotState/CoralEndEffector", logged);
    Logger.recordOutput("RobotState/CoralEndEffector/CurrentState", m_state.getLabel());
    switch(m_state) {
      case INTAKING -> {
        if (logged.hasCoral) {
          m_state = CoralEndEffectorState.IDLE; //currently not working, always thinks it has coral
        }
      }
      default -> {}
    }
    setTarget(m_state.getVoltage());
  }

  public Command setStateIntake() {
    return startEnd(
      () -> {
        this.m_state = CoralEndEffectorState.INTAKING;
      },
      () -> {
        this.m_state = CoralEndEffectorState.IDLE;
      }
    );
  }
  
  public Command setStateScoring() {
    return startEnd(
      () -> this.m_state = CoralEndEffectorState.SCORING,
      () -> this.m_state = CoralEndEffectorState.IDLE
    );
  }

  public Trigger hasCoral() {
    return new Trigger(() -> logged.hasCoral);
  }

  public Command setStateEjecting() {
    return startEnd(
      () -> this.m_state = CoralEndEffectorState.EJECTING,
      () -> this.m_state = CoralEndEffectorState.IDLE
    );
  }

  public Command setStateIdle() {
    return new InstantCommand(() -> this.m_state = CoralEndEffectorState.IDLE);
  }
}
