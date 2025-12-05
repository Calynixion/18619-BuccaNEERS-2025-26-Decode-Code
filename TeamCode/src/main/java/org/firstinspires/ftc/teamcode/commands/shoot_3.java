package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class shoot_3 extends CommandBase {
    private Shooter shooter;
    private double power;
    //eventually add camera tracking for target
    public shoot_3(Shooter shooter, double power){
        this.shooter = shooter;
        this.power = power;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.spin(power);
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
