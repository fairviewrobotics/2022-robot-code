package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants
import frc.robot.subsystems.ShooterElevationSubsystem

class ElevationCommand(val elevation: ShooterElevationSubsystem, targetDist: () -> Double, enc: Encoder): PIDCommand(
    PIDController(
        Constants.shooterElevationP,
        Constants.shooterElevationI,
        Constants.shooterElevationD
    ),
    enc::getDistance,
    targetDist(),
    { output: Double -> elevation.setSpeed(output)},
    elevation){
    
    init{
        controller.setTolerance(
            Constants.shooterElevationPosTolerance,
            Constants.shooterElevationVelocityTolerance
        )
    }

    override fun isFinished(): Boolean{
        return controller.atSetpoint()
    }
}