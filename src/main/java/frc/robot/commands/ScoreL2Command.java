package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreL2Command extends Command {
    private Elevator elevator;

    public ScoreL2Command(Elevator elevator){
        this.elevator = elevator;
    }

    @Override
    public void initialize(){
        elevator.setDistance(Inches.of(this.elevator.level2.getAsDouble()));
    }

    @Override
    public boolean isFinished(){
        return elevator.atDistance(() -> 1.01);
    }
}
