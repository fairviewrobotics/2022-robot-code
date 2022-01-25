// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup

import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.ArcadeDriveCommand
import frc.robot.commands.DebugDriveCommand
import frc.robot.subsystems.DrivetrainSubsystem

import frc.robot.subsystems.*
import frc.robot.commands.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // MARK: Hardware initialization -- anything that needs a port
    val controller0 = XboxController(0)
    val controller1 = XboxController(1)

    val motorFrontLeft = WPI_TalonSRX(Constants.kDrivetrainFrontLeftPort)
    val motorBackLeft = WPI_TalonSRX(Constants.kDrivetrainBackLeftPort)
    val motorFrontRight = WPI_TalonSRX(Constants.kDrivetrainFrontRightPort)
    val motorBackRight = WPI_TalonSRX(Constants.kDrivetrainBackRightPort)

    val leftDrivetrainEncoder = Encoder(Constants.kDrivetrainLeftEncoderPortA, Constants.kDrivetrainLeftEncoderPortB, Constants.kDrivetrainEncoderAReversed)
    val rightDrivetrainEncoder = Encoder(Constants.kDrivetrainRightEncoderPortA, Constants.kDrivetrainRightEncoderPortB, Constants.kDrivetrainEncoderBReversed)

    val gyro = ADXRS450_Gyro()

    // MARK: Controllers and groups
    val leftMotors = MotorControllerGroup(motorFrontLeft, motorFrontRight)
    val rightMotors = MotorControllerGroup(motorBackLeft, motorBackRight)

    // MARK: Subsystems
    val drivetrain = DrivetrainSubsystem(leftMotors, rightMotors, leftDrivetrainEncoder, rightDrivetrainEncoder, gyro)

    init {
        configureButtonBindings()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {

    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    //val autonomousCommand: Command
    //    get() =// An ExampleCommand will run in autonomous
    //        m_autoCommand
}