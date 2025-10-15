package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class CEEStop extends Command {
    public CoralEndEffector coralEndEffector;
    public CEEStop(CoralEndEffector cee) {
        this.coralEndEffector = cee;
    }
    @Override
    public void initialize(){
        this.coralEndEffector.setTarget(Volts.of(0));

    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
