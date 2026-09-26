package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class ShooterTargetSpeed extends Command
{
    DoubleSupplier targetSpeedSupplier;
    double targetSpeed;
    
    /**
     * Claims the Shooter subsystem and stores the supplied target speed source.
     * Allows retrieval of the desired flywheel surface velocity.
     */
    public ShooterTargetSpeed(DoubleSupplier targetSpeedSupplier)
    {
        this.targetSpeedSupplier = targetSpeedSupplier;
        addRequirements(Shooter.getInstance());
    }
    
    /**
     * Reads the target speed and applies it as linear velocity.
     */
    @Override
    public void initialize()
    {
        targetSpeed = targetSpeedSupplier.getAsDouble();
        System.out.println("Shooter targetting speed " + targetSpeed);
        Shooter.getInstance().setEffectiveVelocity(MetersPerSecond.of(targetSpeed));
    }

    /**
     * This command finishes immediately.
     */
    @Override
    public boolean isFinished()
    {
        return true;
    }

    /**
     * No cleanup is required.
     */
    @Override
    public void end(boolean interrupted)
    {
    }
}
