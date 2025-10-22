package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
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
        elevator.setDistance(Distance.ofBaseUnits(32, Inches));
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished(){
        if (elevator.atDistance(() -> 1.01)) {
            cee.getNewSetVoltsCommand(2);
            return true;
        }
        return false;
    }
}
