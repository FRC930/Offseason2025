package frc.robot.operator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.EnumState;
import org.littletonrobotics.junction.Logger;

/**
 * Manages operator selections and interprets driver inputs into intent.
 * This is the "UI state management" layer.
 *
 * <p>Responsibilities:
 * <ul>
 *   <li>Store persistent selections (level, side, mode)</li>
 *   <li>Expose raw input triggers (button pressed)</li>
 *   <li>Expose intent triggers (what operator WANTS, considering mode)</li>
 *   <li>Does NOT know about hardware state or safety</li>
 * </ul>
 *
 * <p>In simulation mode, inputs can be triggered via Shuffleboard toggles.
 */
public class OperatorIntent implements OperatorIntentEvents {

    private final CommandXboxController driver;

    // Stored selections using EnumState for automatic trigger generation
    private final EnumState<ScoringLevel> selectedLevel = new EnumState<>("OperatorIntent/Level", ScoringLevel.L1);
    private final EnumState<ScoringSide> selectedSide = new EnumState<>("OperatorIntent/Side", ScoringSide.LEFT);
    private final EnumState<OperatorMode> mode = new EnumState<>("OperatorIntent/Mode", OperatorMode.TELEOP);

    // Simulation mode flag
    private final boolean simMode;

    /**
     * Creates OperatorIntent with driver controller on specified port.
     *
     * @param driverPort USB port for driver controller (typically 0)
     */
    public OperatorIntent(int driverPort) {
        this.driver = new CommandXboxController(driverPort);
        this.simMode = (Constants.currentMode == Constants.Mode.SIM);
        configureSelections();

        if (simMode) {
            configureSimInputs();
        }
    }

    /** Wire POV/bumpers to store selections. */
    private void configureSelections() {
        // POV buttons store the scoring level (persists until changed)
        driver.povDown().onTrue(setLevel(ScoringLevel.L1));
        driver.povRight().onTrue(setLevel(ScoringLevel.L2));
        driver.povLeft().onTrue(setLevel(ScoringLevel.L3));
        driver.povUp().onTrue(setLevel(ScoringLevel.L4));
    }

    /** Configure SmartDashboard inputs for simulation testing. */
    private void configureSimInputs() {
        // Initialize SmartDashboard values
        SmartDashboard.putBoolean("Sim/Score", false);
        SmartDashboard.putBoolean("Sim/Intake", false);
        SmartDashboard.putBoolean("Sim/Eject", false);
        SmartDashboard.putNumber("Sim/Level", 1);

        // Wire level selector changes to set level
        new Trigger(() -> SmartDashboard.getNumber("Sim/Level", 1) == 1).onTrue(setLevel(ScoringLevel.L1));
        new Trigger(() -> SmartDashboard.getNumber("Sim/Level", 1) == 2).onTrue(setLevel(ScoringLevel.L2));
        new Trigger(() -> SmartDashboard.getNumber("Sim/Level", 1) == 3).onTrue(setLevel(ScoringLevel.L3));
        new Trigger(() -> SmartDashboard.getNumber("Sim/Level", 1) == 4).onTrue(setLevel(ScoringLevel.L4));
    }

    // ==================== STATE MUTATION COMMANDS ====================

    public Command setLevel(ScoringLevel level) {
        return Commands.runOnce(() -> selectedLevel.set(level));
    }

    public Command setSide(ScoringSide side) {
        return Commands.runOnce(() -> selectedSide.set(side));
    }

    public Command setMode(OperatorMode newMode) {
        return Commands.runOnce(() -> mode.set(newMode));
    }

    // ==================== RAW INPUT TRIGGERS ====================

    /** Returns true while the score button is held (controller OR sim toggle). */
    public Trigger scoreButtonPressed() {
        Trigger controllerTrigger = driver.rightTrigger(0.5);
        if (simMode) {
            return controllerTrigger.or(new Trigger(() -> SmartDashboard.getBoolean("Sim/Score", false)));
        }
        return controllerTrigger;
    }

    /** Returns true while the intake button is held (controller OR sim toggle). */
    public Trigger intakeButtonPressed() {
        Trigger controllerTrigger = driver.leftTrigger();
        if (simMode) {
            return controllerTrigger.or(new Trigger(() -> SmartDashboard.getBoolean("Sim/Intake", false)));
        }
        return controllerTrigger;
    }

    /** Returns true while the eject button is held (controller OR sim toggle). */
    public Trigger ejectButtonPressed() {
        Trigger controllerTrigger = driver.rightBumper();
        if (simMode) {
            return controllerTrigger.or(new Trigger(() -> SmartDashboard.getBoolean("Sim/Eject", false)));
        }
        return controllerTrigger;
    }

    // ==================== SELECTION TRIGGERS ====================

    @Override
    public Trigger levelIs(ScoringLevel level) {
        return selectedLevel.is(level);
    }

    @Override
    public Trigger sideIs(ScoringSide side) {
        return selectedSide.is(side);
    }

    public Trigger modeIs(OperatorMode operatorMode) {
        return mode.is(operatorMode);
    }

    // ==================== INTENT TRIGGERS ====================
    // These combine raw input + mode checks to express what the operator WANTS

    /** Returns true when operator wants to score (score button + teleop mode). */
    public Trigger wantsToScore() {
        return scoreButtonPressed().and(modeIs(OperatorMode.TELEOP));
    }

    /** Returns true when operator wants to intake (intake button + teleop mode). */
    public Trigger wantsToIntake() {
        return intakeButtonPressed().and(modeIs(OperatorMode.TELEOP));
    }

    /** Returns true when operator wants to eject (eject button + teleop mode). */
    public Trigger wantsToEject() {
        return ejectButtonPressed().and(modeIs(OperatorMode.TELEOP));
    }

    /** Returns true when operator wants to climb (climb mode active). */
    public Trigger wantsToClimb() {
        return modeIs(OperatorMode.CLIMB);
    }

    // ==================== ACCESSORS ====================

    /** Get the currently selected scoring level. */
    public ScoringLevel getSelectedLevel() {
        return selectedLevel.get();
    }

    /** Get the currently selected scoring side. */
    public ScoringSide getSelectedSide() {
        return selectedSide.get();
    }

    /** Get the current operator mode. */
    public OperatorMode getMode() {
        return mode.get();
    }

    /** Get the driver controller for drive commands. */
    public CommandXboxController getDriver() {
        return driver;
    }

    /** Periodic logging (optional - EnumState already logs on changes). */
    public void periodic() {
        Logger.recordOutput("OperatorIntent/WantsToScore", wantsToScore().getAsBoolean());
        Logger.recordOutput("OperatorIntent/WantsToIntake", wantsToIntake().getAsBoolean());
    }
}
