package frc.robot.commands

import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.WinchSubsystem

/*
Set a ball motor to the speed supplied by the speed lambda.
This command does not halt its execution.
 */
class GateSensored(val system: BallMotorSubsystem, val speed: () -> Double, val sensor: ColorSensorV3) : CommandBase() {
    init {
        addRequirements(system)
    }

    override fun execute() {
        val color = sensor.color
        SmartDashboard.putNumber("color red", color.red)
        SmartDashboard.putNumber("color green", color.green)
        SmartDashboard.putNumber("color blue", color.blue)
        system.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        system.setSpeed(0.0)
    }

    override fun isFinished() = false
}