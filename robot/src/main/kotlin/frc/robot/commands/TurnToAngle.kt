package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem

class TurnToAngle(val drivetrain: DrivetrainSubsystem, val controller: XboxController): CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    val anglePID = PIDController(
        6.2296,
        0.1,
        0.0
    )
}