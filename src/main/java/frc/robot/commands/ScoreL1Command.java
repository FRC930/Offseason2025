package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL1Command extends Command {
    private Elevator elevator;

    public ScoreL1Command(Elevator elevator){
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        elevator.getNewSetDistanceCommand(10);        
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
