package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL3Command extends Command {
    private Elevator elevator;

    public ScoreL3Command(Elevator elevator){
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        elevator.getNewSetDistanceCommand(30);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
