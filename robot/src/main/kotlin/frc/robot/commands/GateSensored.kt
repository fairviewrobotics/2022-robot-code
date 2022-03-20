package frc.robot.commands

import com.revrobotics.ColorMatch
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.WinchSubsystem
import java.sql.Driver

/*
Set a ball motor to the speed supplied by the speed lambda.
This command does not halt its execution.
 */
class GateSensored(val system: BallMotorSubsystem, val speed: () -> Double, val sensor: ColorSensorV3) : CommandBase() {
    val colorMatcher = ColorMatch()

    var ourColor = Color(0.0, 0.0, 0.0)
    var enemyColor = Color(0.0, 0.0, 0.0)

    val blueTarget = Color(0.143, 0.427, 0.429)
    val redTarget = Color(0.143, 0.427, 0.429)

    init {
        addRequirements(system)

        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            ourColor = blueTarget
            enemyColor = redTarget
        } else {
            ourColor = redTarget
            enemyColor = blueTarget
        }

        colorMatcher.addColorMatch(ourColor)
        colorMatcher.addColorMatch(enemyColor)
    }

    override fun execute() {
        val color = sensor.color
        SmartDashboard.putNumber("color red", color.red)
        SmartDashboard.putNumber("color green", color.green)
        SmartDashboard.putNumber("color blue", color.blue)
        SmartDashboard.putNumber("proximity", sensor.proximity.toDouble())

        SmartDashboard.putBoolean("our ball",
            colorMatcher.matchClosestColor(sensor.color) == ourColor
        )

        if (sensor.proximity < Constants.kColorSensorProximityThreshold) {
            system.setSpeed(speed())
        } else {
            system.setSpeed(0.0)
        }

    }

    override fun end(interrupted: Boolean) {
        system.setSpeed(0.0)
    }

    override fun isFinished() = false
}