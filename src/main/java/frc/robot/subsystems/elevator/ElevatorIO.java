package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import frc.robot.util.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorInputs {
        public MutDistance distance = Inches.mutable(0);
        public MutLinearVelocity velocity = InchesPerSecond.mutable(0);
        public MutDistance setPoint = Inches.mutable(0);
        public MutVoltage voltage = Volts.mutable(0);
        public MutVoltage voltageSetPoint = Volts.mutable(0);
        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }

    public void updateInputs(ElevatorInputs input);

    public void stop();

    public void setTarget(Distance distance);

    public void setGains(Gains gains);
}
