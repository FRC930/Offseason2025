package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class CoralEndEffectorStop extends Command {
    public CoralEndEffector coralEndEffector;
    public CoralEndEffectorStop(CoralEndEffector cee) {
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
