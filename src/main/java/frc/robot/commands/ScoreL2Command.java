package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL2Command extends Command {
    private Elevator elevator;

    public ScoreL2Command(Elevator elevator){
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        elevator.getNewSetDistanceCommand(20);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
