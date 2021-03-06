package frc.robot.commands

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem

class DebugClimbingCommand(val climber: WinchSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(climber)
    }

    var speed = 0.0;
    override fun execute() {
        if (controller.getAButton()) {
            speed += 0.1
        }

        if (controller.getBButton()) {
            speed -= 0.1
        }

        if (controller.xButtonPressed) {
            climber.resetEncoder()
        }

        climber.setVoltage(speed)
    }
}