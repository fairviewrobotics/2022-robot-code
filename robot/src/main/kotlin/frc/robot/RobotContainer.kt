// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*
import edu.wpi.first.wpilibj.PneumaticsModuleType
import com.revrobotics.*
import edu.wpi.first.wpilibj.DigitalInput


import edu.wpi.first.wpilibj.XboxController.Button.*


import frc.robot.subsystems.*


import frc.robot.commands.*

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Hardware and subsystem initialization

    val controller0 = XboxController(0)
    val controller1 = XboxController(1)

    // drivetrain
    val motorFrontLeft = CANSparkMax(Constants.driveFrontLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackLeft = CANSparkMax(Constants.driveBackLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorFrontRight = CANSparkMax(Constants.driveFrontRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackRight = CANSparkMax(Constants.driveBackRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val drivetrain = CANSparkMaxDrivetrainSubsystem(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, AHRS())

    // climber
    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    // TODO: attach limit switches directly to pins on Spark
    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1))

    val leftSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidLeftID.first,Constants.climbSolenoidLeftID.second)
    val rightSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidRightID.first,Constants.climbSolenoidRightID.second)
    val climbLeft = GenericSolenoidSubsystem(leftSolenoid)
    val climbRight = GenericSolenoidSubsystem(rightSolenoid)

    // shooter
    val shooterMotor1 = CANSparkMax(Constants.shooterLowID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val shooterMotor2  = CANSparkMax(Constants.shooterHighID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val shooter1 = ShooterSubsystem(shooterMotor1)
    val shooter2 = ShooterSubsystem(shooterMotor2)

    // intake / indexer / gate
    val intake = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))
    val indexer = BallMotorSubsystem(WPI_TalonSRX(Constants.indexerID))
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))
    
    // simultaneous pneumatics push and pull
    val climberPull = ParallelCommandGroup(
        GenericPneumaticCommand(climbLeft, kReverse).withTimeout(1.0),
        GenericPneumaticCommand(climbRight, kReverse).withTimeout(1.0)
    )

    val climberPush = ParallelCommandGroup(
        GenericPneumaticCommand(climbLeft, kForward).withTimeout(1.0),
        GenericPneumaticCommand(climbRight, kForward).withTimeout(1.0)
    )


    // initial climb: drive up, lower climber fully, pull in pneumatics
    val initialClimb = SequentialCommandGroup(
        LimitedWinchCommand(winch, { -1.0 }),
        climberPull
    )

    // secondary climb: raise climber to half
    // ! all times are in seconds
    val secondaryClimb = SequentialCommandGroup(
        FixedWinchSpeed(winch, { 1.0 }).withTimeout(0.5), // todo: tune
        climberPull,
        LimitedWinchCommand(winch, { 1.0 }),
        climberPush,
        FixedWinchSpeed(winch, { -1.0 }).withTimeout(0.5),
        LimitedWinchCommand(winch, { -1.0 })
    )


    init {
        configureButtonBindings()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        // run shooter when bumpers are held
        JoystickButton(controller0, kRightBumper.value).whenHeld(
            ShooterBangBang(shooter1, { Constants.shooterRPM })
        )

        JoystickButton(controller0, kLeftBumper.value).whenHeld(
            ShooterPID(shooter1, { Constants.shooterRPM })
        )

        JoystickButton(controller0, kX.value).whenHeld(
            FixedShooterSpeed(shooter1, { 1.0 })
        )

        // run intake on A
        JoystickButton(controller0, kA.value).whenHeld(
            FixedBallMotorSpeed(intake, { Constants.intakeSpeed })
        )

        // run intake + indexer on B
        JoystickButton(controller0, kB.value).whenHeld(
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed }),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed })
            )
        )

        // run gate on Y
        JoystickButton(controller0, kY.value).whenHeld(
            FixedBallMotorSpeed(gate, { Constants.gateSpeed })
        )

        drivetrain.defaultCommand = DirectJoystickDrive(drivetrain, controller0)
        //drivetrain.defaultCommand = JoystickDrive(drivetrain, controller0)
        //debugSubsystem.defaultCommand = MotorTest(debugSubsystem, controller0)
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