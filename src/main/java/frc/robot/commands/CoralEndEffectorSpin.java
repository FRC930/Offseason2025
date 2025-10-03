package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;

public class CoralEndEffectorSpin extends Command {
    public CoralEndEffector coralEndEffector;
    public CoralEndEffectorSpin(CoralEndEffector cee) {
        this.coralEndEffector = cee;
    }
    @Override
    public void initialize(){
    this.coralEndEffector.getNewSetVoltsCommand(2);

    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
