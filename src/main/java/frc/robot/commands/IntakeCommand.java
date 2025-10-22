package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class IntakeCommand extends Command {
    private CoralEndEffector cee;

    public IntakeCommand(CoralEndEffector cee){
        this.cee = cee;
    }

    @Override
    public void initialize(){
        cee.getNewSetVoltsCommand(2);
    }    

    @Override
    public boolean isFinished(){
        return cee.getNewHasCoralSupplier().getAsBoolean();
    }
}
