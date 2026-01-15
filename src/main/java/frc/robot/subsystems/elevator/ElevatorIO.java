package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.util.Gains;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorInputs {
    public MutDistance distance;
    public MutLinearVelocity velocity;
    public MutDistance setPoint;
    public MutVoltage voltage;
    public MutVoltage voltageSetPoint;
    public MutCurrent supplyCurrent;
    public MutCurrent torqueCurrent;
  }

  public void updateInputs(ElevatorInputs input);

  public void stop();

  public void setTarget(Distance distance);

  public void setGains(Gains gains);

  public void setVoltage(double voltage);
}
