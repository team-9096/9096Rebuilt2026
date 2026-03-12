package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkSoftLimit.SoftLimitDirection;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private SparkMaxConfig pivotConfig;
    private SparkMax pivot;
    private SparkMax intake;

    public Intake() {
        pivotConfig = new SparkMaxConfig();
        pivotConfig.softLimit.forwardSoftLimit(0.5);
        pivotConfig.softLimit.reverseSoftLimit(0);
        pivotConfig.idleMode(IdleMode.kCoast);
        pivot = new SparkMax(Constants.MechanismConstants.INTAKE_PIVOT, MotorType.kBrushless);
        pivot.configure(pivotConfig, null, PersistMode.kPersistParameters);
        intake = new SparkMax(Constants.MechanismConstants.INTAKE_MOTOR, MotorType.kBrushless);
    }
    
    public boolean pivotOpen() {
        return pivot.getForwardSoftLimit().isReached();
    }

    public boolean pivotClosed() {
        return pivot.getReverseSoftLimit().isReached();
    }

    public void pivot(double speed) {
        pivot.set(speed);
    }

    public void intake(double speed) {
        intake.set(speed);
    }

}
