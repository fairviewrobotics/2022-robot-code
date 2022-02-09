package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem
import java.lang.Math.atan

class TurnToAngle(val drivetrain: DrivetrainSubsystem, val controller: XboxController): CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    val kinematics = DifferentialDriveKinematics(21.5)

    override fun execute() {
        SmartDashboard.putNumber("Gyro", drivetrain.heading)

    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0,0.0)
    }
}