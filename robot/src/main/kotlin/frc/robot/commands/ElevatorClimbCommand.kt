package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants
import frc.robot.subsystems.WinchSubsystem


class ElevatorClimbCommand(val climber: WinchSubsystem,
                           val targetPosition: () -> Double): PIDCommand(
    PIDController(
        Constants.elevatorP,
        Constants.elevatorI,
        Constants.elevatorD
    ),
    climber::getPosition,
    targetPosition,
    { output: Double -> climber.setVoltage(output) },
    climber
)