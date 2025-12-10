package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableGainsBuilder;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  private ElevatorIO m_ElevatorIO;

  ElevatorInputsAutoLogged loggedelevator = new ElevatorInputsAutoLogged();

  public static final double SPOOL_RADIUS = 1.751 / 2.0;

  public static final double INCHES_PER_ROT = (2.0 * (2.0 * Math.PI * SPOOL_RADIUS)); //Multiplied by 2 because its a 2 stage cascading elevator
  
  public static final double REDUCTION = (11.0/4.0);

  public LoggedTunableGainsBuilder tunableGains = new LoggedTunableGainsBuilder(
    "Gains/Elevator/", 
    400.0, 0, 25.0, 
    0.0, 30.0, 0.0, 0.0, 
    75.0, 30.0, 0.0, 0.0, 0.0
  );

  // LoggedTunableNumbers for Levels
  public LoggedTunableNumber level1 = new LoggedTunableNumber(
    "Elevator/Levels/L1",
    0
  );
  
  public LoggedTunableNumber level2 = new LoggedTunableNumber(
    "Elevator/Levels/L2",
    0
  );

  public LoggedTunableNumber level3 = new LoggedTunableNumber(
    "Elevator/Levels/L3",
    0
  );

  public LoggedTunableNumber level4 = new LoggedTunableNumber(
    "Elevator/Levels/L4",
    0
  );

  public Elevator(ElevatorIO elevatorIO) {
    m_ElevatorIO = elevatorIO;
    loggedelevator.distance = Inches.mutable(0);
    loggedelevator.velocity = InchesPerSecond.mutable(0);
    loggedelevator.setPoint = Meters.mutable(0);
    loggedelevator.supplyCurrent = Amps.mutable(0);
    loggedelevator.torqueCurrent = Amps.mutable(0);
    loggedelevator.voltageSetPoint = Volts.mutable(0);
    loggedelevator.voltage = Volts.mutable(0);

    this.m_ElevatorIO.setGains(tunableGains.build());
    RobotState.instance().setElevatorSource(loggedelevator.distance);
  }

  public Supplier<Distance> getDistanceExtendedSupplier() {
    return () -> loggedelevator.distance;
  }

  public Supplier<Boolean> getDistanceGreaterSupplier(Distance distance) {
    return () -> loggedelevator.distance.baseUnitMagnitude() > distance.baseUnitMagnitude();
  }

  public Supplier<Boolean> getDistanceLessSupplier(Distance distance) {
    return () -> loggedelevator.distance.baseUnitMagnitude() < distance.baseUnitMagnitude();
  }

  public Supplier<Boolean> getDistanceAtSupplier(Distance distance, Distance error) {
    return () -> MathUtil.isNear(distance.baseUnitMagnitude(), loggedelevator.distance.baseUnitMagnitude(), error.baseUnitMagnitude());
  }

  public void setDistance(Distance target) {
    m_ElevatorIO.setTarget(target);
  }

  public Command getNewSetDistanceCommand(DoubleSupplier distance) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(distance.getAsDouble()));
        },
        this);
  }
  /**
   * @param i Inches
   * */
  public Command getNewSetDistanceCommand(double i) {
    return new InstantCommand(
        () -> {
          setDistance(Inches.of(i));
        },
        this);
  }

  

  public Trigger getNewAtDistanceTrigger(Distance dist, Distance tolerance) {
    return new Trigger(() -> {
      return MathUtil.isNear(dist.baseUnitMagnitude(), loggedelevator.distance.baseUnitMagnitude(), tolerance.baseUnitMagnitude());
    });
  }

  public Trigger getNewAtDistanceTrigger(DoubleSupplier dist, DoubleSupplier tolerance) {
    return new Trigger(() -> {
      return atDistance(dist, tolerance);
    });
  }

  public boolean atDistance(DoubleSupplier tolerance) {
    return MathUtil.isNear(loggedelevator.setPoint.in(Inches), loggedelevator.distance.in(Inches), tolerance.getAsDouble());
  }

  public boolean atDistance(DoubleSupplier dist, DoubleSupplier tolerance) {
    return MathUtil.isNear(dist.getAsDouble(), loggedelevator.distance.in(Inches), tolerance.getAsDouble());
  }

  /**
   * Returns when this joint is greater than 'angle' away from the forward horizontal
   * @param angle
   * @return
   */
  public Trigger getNewGreaterThanDistanceTrigger(DoubleSupplier distance) {
    return new Trigger(() -> {
      return loggedelevator.distance.in(Inches) > distance.getAsDouble();
    });
  }

  /**
   * Returns when this joint is less than 'angle' away from the forward horizontal
   * @param angle
   * @return
   */
  public Trigger getNewLessThanDistanceTrigger(DoubleSupplier distance) {
    return new Trigger(() -> {
      return loggedelevator.distance.in(Inches) < distance.getAsDouble();
    });
  }

  @Override
  public void periodic() {
    tunableGains.ifGainsHaveChanged((gains) -> this.m_ElevatorIO.setGains(gains));
    m_ElevatorIO.updateInputs(loggedelevator);
    Logger.processInputs("RobotState/Elevator", loggedelevator);
  }
}
