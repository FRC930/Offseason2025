package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public enum CoralEndEffectorState {
    IDLE(Volts.of(0.0)),
    INTAKING(Volts.of(4.0)),
    SCORING(Volts.of(12.0)),
    EJECTING(Volts.of(-12.0));

    private Voltage m_voltage;
    private CoralEndEffectorState(Voltage voltage) {
        m_voltage = voltage;
    }
    
    public Voltage getVoltage() {
        return m_voltage;
    }
}