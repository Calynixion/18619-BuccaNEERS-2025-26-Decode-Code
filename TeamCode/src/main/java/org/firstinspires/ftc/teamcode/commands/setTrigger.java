package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Bot_Trigger;

public class setTrigger extends CommandBase {
    private final double power;
    private final Bot_Trigger trigger;

    public setTrigger(Bot_Trigger trigger, double power){
        this.power=power;
        this.trigger=trigger;
        addRequirements(trigger);
    }

    @Override
    public void execute(){
        trigger.setPower(power);
    }

}
