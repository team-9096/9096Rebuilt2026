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
    }

    @Override
    public void execute() {
        if(!intakeSubsystem.pivotOpen() && pivotOut.getAsBoolean()) {
            intakeSubsystem.pivot(0.5);
        }
        else if (!intakeSubsystem.pivotClosed() && pivotIn.getAsBoolean()) {
            intakeSubsystem.pivot(-0.5);
        }
        else {
            intakeSubsystem.pivot(0);
        }

        if (intake.getAsBoolean()) {
            intakeSubsystem.intake(1);
        }
    }

}
