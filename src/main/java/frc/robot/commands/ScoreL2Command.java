package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

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
        elevator.setDistance(Inches.of(20));
    }

    @Override
    public boolean isFinished(){
        return elevator.atDistance(() -> 1.01);
    }
}
