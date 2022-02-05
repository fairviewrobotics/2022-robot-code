package frc.robot.commands

import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.DrivetrainSubsystem
import kotlin.math.max
import kotlin.math.min

/**
 * various debug commands used for testing and diagnostic
 * very freeform and not very refined
 */
class DebugDrive(val drivetrain: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    val kinematics = DifferentialDriveKinematics(21.5)

    var csv = "time,setpoint,actual"
    var t = 0.0
    override fun execute() {
        t += 1/50
        val speeds = kinematics.toWheelSpeeds(ChassisSpeeds(-controller.leftY, 0.0, controller.leftX))

        val desiredLeft = speeds.leftMetersPerSecond
        val desiredRight = speeds.rightMetersPerSecond

        val actual = drivetrain.wheelSpeeds.leftMetersPerSecond
        val newSpeeds = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(desiredLeft, desiredRight))
        csv += "\n${t},${desiredLeft},${actual}"

        drivetrain.drive.arcadeDrive(newSpeeds.vxMetersPerSecond, newSpeeds.omegaRadiansPerSecond)
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putString("DATA DUMP!!!!!", csv)

    }

    override fun isFinished() = false
}

class DebugDrive2Subsystem(val motor: TalonFX) : SubsystemBase() {
    fun volts(volts: Double) {
        motor.set(TalonFXControlMode.PercentOutput, volts)
    }

    override fun periodic() {
        super.periodic()
    }

}
class DebugDrive2(val subsystem: DebugDrive2Subsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(subsystem)
    }
    var state = 0.0

    override fun execute() {
        if (controller.aButtonPressed) {
            state += 0.1
        }

        if (controller.bButtonPressed) {
            state -= 0.1
        }


        subsystem.volts(state);
    }

    override fun isFinished() = false

}