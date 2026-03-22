package frc.robot.commands.Drivebase;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Intake intakeSubsystem;
    private final BooleanSupplier pivotIn;
    private final BooleanSupplier pivotOut;
    private final BooleanSupplier intake;

    public IntakeCommand (Intake intakeSubsystem, BooleanSupplier pivotIn, BooleanSupplier pivotOut, BooleanSupplier intake) {
        this.pivotIn = pivotIn;
        this.pivotOut = pivotOut;
        this.intake = intake;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if(!intakeSubsystem.pivotOpen() && pivotOut.getAsBoolean()) {
            intakeSubsystem.pivot(0.25);
        }
        else if (!intakeSubsystem.pivotClosed() && pivotIn.getAsBoolean()) {
            intakeSubsystem.pivot(-0.25);
        }
        else {
            intakeSubsystem.pivot(0);
        }

        if (intake.getAsBoolean()) {
            intakeSubsystem.intake(.5);
        }
        else {
            intakeSubsystem.intake(0); //Disable intake speed when button is not pressed
        }
    }

}
