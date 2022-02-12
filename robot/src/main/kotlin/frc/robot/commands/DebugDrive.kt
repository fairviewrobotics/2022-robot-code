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

/**
 * various debug commands used for testing and diagnostic
 * very freeform and not very refined
 */
class DebugDrive(val drivetrain: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        if (controller.aButtonPressed) {
            drivetrain.gyro.reset()
        }
        SmartDashboard.putNumber("Angle (Degrees)", drivetrain.degrees)
    }

    override fun end(interrupted: Boolean) {
    }

    override fun isFinished() = false
}

class MotorTestSubsystem(val motor: TalonFX) : SubsystemBase() {
    fun volts(volts: Double) {
        motor.set(TalonFXControlMode.PercentOutput, volts)
    }

    override fun periodic() {
        super.periodic()
    }

}
class MotorTest(val subsystem: MotorTestSubsystem, val controller: XboxController) : CommandBase() {
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