package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL4Command extends Command {
    private Elevator elevator;
    private CoralEndEffector cee;

    public ScoreL4Command(Elevator elevator, CoralEndEffector cee){
        this.elevator = elevator;
        this.cee = cee;
    }

    @Override
    public void initialize(){
        elevator.setDistance(Inches.of(40));
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
