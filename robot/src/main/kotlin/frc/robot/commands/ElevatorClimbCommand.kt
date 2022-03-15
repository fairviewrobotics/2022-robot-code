package frc.robot.commands

import edu.wpi.first.math.controller.PIDController


class ElevatorClimbCommand(val climber: WinchSubsystem,
                           val targetPosition: () -> Double): PIDCommand(
    PIDController(
        Constants.shooterElevationP,
        Constants.shooterElevationI,
        Constants.shooterElevationD
    ),
    climber::getPosition,
    targetPosition,
    { output: Double -> climber.setVoltage(output) },
    climber
)