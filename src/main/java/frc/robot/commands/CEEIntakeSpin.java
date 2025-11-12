package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class CEEIntakeSpin extends Command {
    public CoralEndEffector coralEndEffector;
    public CEEIntakeSpin(CoralEndEffector cee) {
        this.coralEndEffector = cee;
    }
    @Override
    public void initialize(){
        this.coralEndEffector.setTarget(Volts.of(0.5));

    }
    @Override
    public boolean isFinished() {
        if(coralEndEffector.getNewHasCoralSupplier().getAsBoolean()) {
            coralEndEffector.setTarget(Volts.of(0.0));
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        coralEndEffector.setTarget(Volts.of(0.0));
        super.end(interrupted);
    }
}
