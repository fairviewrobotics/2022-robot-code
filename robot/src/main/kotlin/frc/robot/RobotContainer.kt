// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.DrivetrainSubsystem
import frc.robot.subsystems.DrivetrainSubsystemConstants

object RobotContainerConstants {
    val drivetrainFrontLeftPort = 0;
    val drivetrainFrontRightPort = 1;
    val drivetrainBackLeftPort = 2;
    val drivetrainBackRightPort = 3;

    val drivetrainLeftEncoderPortA = 4;
    val drivetrainLeftEncoderPortB = 5;
    val drivetrainRightEncoderPortA = 6;
    val drivetrainRightEncoderPortB = 7;

    val drivetrainEncoderAReversed = true;
    val drivetrainEncoderBReversed = true;
}

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // val m_autoCommand: Command =
    val controller0 = XboxController(1)
    /* controller1 - primary driver controller (overriden by controller0) */
    val controller1 = XboxController(0)

    /** --- setup drivetrain --- **/
    val motorFrontLeft = WPI_TalonSRX(Constants.drivetrainFrontLeftPort)
    val motorBackLeft = WPI_TalonSRX(Constants.drivetrainBackLeftPort)
    val motorFrontRight = WPI_TalonSRX(Constants.drivetrainFrontRightPort)
    val motorBackRight = WPI_TalonSRX(Constants.drivetrainBackRightPort)

    val leftMotors = MotorControllerGroup(motorFrontLeft, motorFrontRight)
    val rightMotors = MotorControllerGroup(motorFrontRight, motorFrontRight)

    val leftDrivetrainEncoder = Encoder(RobotContainerConstants.drivetrainLeftEncoderPortA, RobotContainerConstants.drivetrainLeftEncoderPortB, RobotContainerConstants.drivetrainEncoderAReversed)
    val rightDrivetrainEncoder = Encoder(RobotContainerConstants.drivetrainRightEncoderPortA, RobotContainerConstants.drivetrainRightEncoderPortB, RobotContainerConstants.drivetrainEncoderBReversed)

    val gyro = ADXRS450_Gyro()

    val drivetrain = DrivetrainSubsystem(leftMotors, rightMotors, leftDrivetrainEncoder, rightDrivetrainEncoder, gyro)
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        // Configure the button bindings
        configureButtonBindings()
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {}// An ExampleCommand will run in autonomous

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    //val autonomousCommand: Command
    //    get() =// An ExampleCommand will run in autonomous
    //        m_autoCommand
}