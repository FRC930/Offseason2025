package frc.robot.operator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Interprets driver controller inputs into intent triggers.
 *
 * This class has NO state - it just wraps controller buttons
 * and provides semantic meaning (wantsToScore vs rightTrigger).
 *
 * State lives in RobotGoals. This class just says what the
 * operator is physically doing right now.
 *
 * In simulation mode, inputs can be triggered via SmartDashboard toggles.
 */
public class OperatorIntent implements OperatorIntentEvents {

    private final CommandXboxController driver;
    private final boolean simMode;

    public OperatorIntent(int driverPort) {
        this.driver = new CommandXboxController(driverPort);
        this.simMode = (Constants.currentMode == Constants.Mode.SIM);

        if (simMode) {
            initializeSimDashboard();
        }
    }

    private void initializeSimDashboard() {
        SmartDashboard.putBoolean("Sim/Score", false);
        SmartDashboard.putBoolean("Sim/Intake", false);
        SmartDashboard.putBoolean("Sim/Eject", false);
        SmartDashboard.putNumber("Sim/Level", 1);
    }

    // ==================== RAW BUTTON TRIGGERS ====================

    public Trigger wantsToScore() {
        Trigger controllerTrigger = driver.rightTrigger(0.5);
        if (simMode) {
            return controllerTrigger.or(simTrigger("Sim/Score"));
        }
        return controllerTrigger;
    }

    public Trigger wantsToIntake() {
        Trigger controllerTrigger = driver.leftTrigger(0.5);
        if (simMode) {
            return controllerTrigger.or(simTrigger("Sim/Intake"));
        }
        return controllerTrigger;
    }

    public Trigger wantsToEject() {
        Trigger controllerTrigger = driver.rightBumper();
        if (simMode) {
            return controllerTrigger.or(simTrigger("Sim/Eject"));
        }
        return controllerTrigger;
    }

    // ==================== LEVEL SELECTION BUTTONS ====================

    public Trigger selectL1() {
        Trigger controllerTrigger = driver.povDown();
        if (simMode) {
            return controllerTrigger.or(simLevelTrigger(1));
        }
        return controllerTrigger;
    }

    public Trigger selectL2() {
        Trigger controllerTrigger = driver.povRight();
        if (simMode) {
            return controllerTrigger.or(simLevelTrigger(2));
        }
        return controllerTrigger;
    }

    public Trigger selectL3() {
        Trigger controllerTrigger = driver.povLeft();
        if (simMode) {
            return controllerTrigger.or(simLevelTrigger(3));
        }
        return controllerTrigger;
    }

    public Trigger selectL4() {
        Trigger controllerTrigger = driver.povUp();
        if (simMode) {
            return controllerTrigger.or(simLevelTrigger(4));
        }
        return controllerTrigger;
    }

    // ==================== SIM HELPERS ====================

    private Trigger simTrigger(String key) {
        return new Trigger(() -> SmartDashboard.getBoolean(key, false));
    }

    private Trigger simLevelTrigger(int level) {
        return new Trigger(() -> (int) SmartDashboard.getNumber("Sim/Level", 1) == level);
    }

    // ==================== ACCESSORS ====================

    public CommandXboxController getDriver() {
        return driver;
    }
}
