// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.can.TalonFX
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
    // MARK: Hardware initialization -- anything that needs a port
    val controller0 = XboxController(0)
    val controller1 = XboxController(1)


    val motorFrontLeft = WPI_TalonSRX(1)
    val motorBackLeft = WPI_TalonSRX(2)
    val motorFrontRight = WPI_TalonSRX(3)
    val motorBackRight = WPI_TalonSRX(4)

    val gyro = AHRS()


    // climber hardware
    val winchMotor = PWMTalonFX(1) // TODO: change
    val lowerLimit = DigitalInput(1)
    val upperLimit = DigitalInput(2)

    val compressor = Compressor(PneumaticsModuleType.CTREPCM)
    val leftSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1,2)
    val rightSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3,4)

    // shooter hardware
    val shooterLower = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
    val shooterUpper  = CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless)

    // intake / indexer hardware
    val indexMotor = PWMTalonSRX(5)

    // MARK: Subsystems
    val drivetrain = TalonSRXDrivetrainSubsystem(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, gyro, 1, 2, 3, 4, 400.0)

    // climber
    val winch = WinchSubsystem(winchMotor, lowerLimit, upperLimit)
    val compressorSys = CompressorSubsystem(compressor)
    val climbLeft = GenericSolenoidSubsystem(leftSolenoid)
    val climbRight = GenericSolenoidSubsystem(rightSolenoid)
    
    // shooter
    val shooter = ShooterSubsystem(shooterLower, shooterUpper)
    

    // intake / indexer
    val indexer = IndexerSubsystem(indexMotor)
    
    // simultaneous pneumatics push and pull
    val climberPull = ParallelCommandGroup(
        GenericPneumaticCommand(climbLeft, compressorSys, kReverse).withTimeout(1.0),
        GenericPneumaticCommand(climbRight, compressorSys, kReverse).withTimeout(1.0)
    )

    val climberPush = ParallelCommandGroup(
        GenericPneumaticCommand(climbLeft, compressorSys, kForward).withTimeout(1.0),
        GenericPneumaticCommand(climbRight, compressorSys, kForward).withTimeout(1.0)
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

        JoystickButton(controller0, kRightBumper.value).whenHeld(
            ShooterBangBang(shooter, { 1.0 })
        )

        JoystickButton(controller0, kLeftBumper.value).whenHeld(
            FixedIndexerSpeed(indexer, {1.0})
        )
        /*JoystickButton(controller0, XboxController.Button.kA.value).whenPressed(
            ArcadePIDDrive(drivetrain, controller0)
        )

        JoystickButton(controller0, XboxController.Button.kB.value).whenPressed(
            ArcadeDriveCommand(drivetrain, controller0)
        )*/

        //drivetrain.defaultCommand = DirectDebugDrive(drivetrain, controller0)
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