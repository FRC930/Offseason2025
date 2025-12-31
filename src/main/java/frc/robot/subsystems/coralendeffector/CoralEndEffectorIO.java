package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface CoralEndEffectorIO {

    @AutoLog
    public static class CoralEndEffectorInputs {
        public MutAngularVelocity angularVelocity = DegreesPerSecond.mutable(0);
        public MutVoltage voltage = Volts.mutable(0);
        public MutVoltage voltageSetPoint = Volts.mutable(0);
        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
        public MutDistance coralDistance = Inches.mutable(100);
        public double rangeStrength = 0;
        public boolean hasCoralDist = false;
        public boolean hasCoral = false;
    }

    public void setTarget(Voltage target);

    /**
     * Takes a set of inputs, retrieves the current values of these inputs, then updates the given
     * input set.
     *
     * <p>
     */
    public void updateInputs(CoralEndEffectorInputs input);

    public void stop();
}
