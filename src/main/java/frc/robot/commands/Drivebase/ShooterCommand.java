package frc.robot.commands.Drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
    private final Shooter shooter;
    private final BooleanSupplier shoot;
    private final DoubleSupplier shotSpeed;

    public ShooterCommand(Shooter shooter, BooleanSupplier shoot, DoubleSupplier shotSpeed) {
        this.shooter = shooter;
        this.shoot = shoot;
        this.shotSpeed = shotSpeed;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if(shoot.getAsBoolean()) {
            shooter.shoot(-1);
        }
        else {
            shooter.shoot(-shotSpeed.getAsDouble());
        }
    }
}
