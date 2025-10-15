package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL4Command extends Command {
    private Elevator elevator;

    public ScoreL4Command(Elevator elevator){
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        elevator.getNewSetDistanceCommand(40);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
