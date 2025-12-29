package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
 */
public class OperatorIntent implements OperatorIntentEvents {

    private final CommandXboxController driver;

    // Stored selections using EnumState for automatic trigger generation
    private final EnumState<ScoringLevel> selectedLevel = new EnumState<>("OperatorIntent/Level", ScoringLevel.L1);
    private final EnumState<ScoringSide> selectedSide = new EnumState<>("OperatorIntent/Side", ScoringSide.LEFT);
    private final EnumState<OperatorMode> mode = new EnumState<>("OperatorIntent/Mode", OperatorMode.TELEOP);

    /**
     * Creates OperatorIntent with driver controller on specified port.
     *
     * @param driverPort USB port for driver controller (typically 0)
     */
    public OperatorIntent(int driverPort) {
        this.driver = new CommandXboxController(driverPort);
        configureSelections();
    }

    /** Wire POV/bumpers to store selections. */
    private void configureSelections() {
        // POV buttons store the scoring level (persists until changed)
        driver.povDown().onTrue(setLevel(ScoringLevel.L1));
        driver.povRight().onTrue(setLevel(ScoringLevel.L2));
        driver.povLeft().onTrue(setLevel(ScoringLevel.L3));
        driver.povUp().onTrue(setLevel(ScoringLevel.L4));
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

    /** Returns true while the score button is held. */
    public Trigger scoreButtonPressed() {
        return driver.rightTrigger(0.5);
    }

    /** Returns true while the intake button is held. */
    public Trigger intakeButtonPressed() {
        return driver.leftTrigger();
    }

    /** Returns true while the eject button is held. */
    public Trigger ejectButtonPressed() {
        return driver.rightBumper();
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
