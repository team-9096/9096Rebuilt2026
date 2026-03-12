package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkMax shooter;

    public Shooter() {
        shooter = new SparkMax(Constants.MechanismConstants.SHOOTER, MotorType.kBrushless);
    }

    public void shoot(double speed) {
        shooter.set(speed);
    }
}
