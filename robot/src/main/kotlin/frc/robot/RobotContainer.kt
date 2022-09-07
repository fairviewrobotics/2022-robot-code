// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import CheckVisionOrRumble
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj.DigitalInput

import com.revrobotics.*
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger

import frc.robot.subsystems.*
import frc.robot.commands.*
import java.lang.Math.PI

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // Hardware and subsystem initialization
    val primaryController = XboxController(0)
    val secondaryController = XboxController(1)

    // drivetrain
    val motorFrontLeft = CANSparkMax(Constants.driveFrontLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackLeft = CANSparkMax(Constants.driveBackLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorFrontRight = CANSparkMax(Constants.driveFrontRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val motorBackRight = CANSparkMax(Constants.driveBackRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val drivetrain = CANSparkMaxDrivetrainSubsystem(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, AHRS())

    // climber
//    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
//    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1), true)

//    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidID.first,Constants.climbSolenoidID.second)
//    val intakeSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolenoidID.first, Constants.intakeSolenoidID.second)
//    val climbPneumatics = SolenoidSubsystem(climbSolenoid)
//    val intakePneumatics = SolenoidSubsystem(intakeSolenoid)

    // shooter

    val shooterMotor1 = WPI_TalonFX(Constants.shooterLowID)
    val shooterMotor2  = WPI_TalonFX(Constants.shooterHighID)
    val shooter1 = TalonFXShooterSubsystem(shooterMotor1, 1.0)
    val shooter2 = TalonFXShooterSubsystem(shooterMotor2, 1.0)
    // intake / indexer / gate
    //val intake = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))
    val indexer = BallMotorSubsystem(WPI_TalonSRX(Constants.indexerID))
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))

    // gate color sensor
    val colorSensor = ColorSensorV3(I2C.Port.kOnboard)

    // auto command chooser
    var autoCommandChooser: SendableChooser<Command> = SendableChooser()

    init {
        configureButtonBindings()
        configureAutoOptions()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        val runGateForward = { FixedBallMotorSpeed(gate, { Constants.gateSpeed}) }
        val runGateBackward = { FixedBallMotorSpeed(gate, { -Constants.gateSpeed}) }
        val runIndexerForward = { FixedBallMotorSpeed(indexer, { -Constants.indexerSpeed}) }
        val runIndexerBackward = { FixedBallMotorSpeed(indexer, { -Constants.indexerSpeed}) }

        val turnToAngleOnDpad = { controller: XboxController ->
            // D-Pad: Turn to fixed angle
            for (i in 0 until 8) {
                val angleDeg = 45 * i
                POVButton(controller, angleDeg).whenHeld(
                    TurnToAngle(drivetrain, { angleDeg * PI / 180.0 })
                )
            }
        }
        
        val fixedSpeedShooter = {
            ParallelCommandGroup(
                DualShooterPID(shooter1, shooter2) { DualShootSpeed(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS) },
                ShootBallMotor(shooter1, shooter2, gate, indexer),
                MaintainAngle(drivetrain)
            )
        }
        drivetrain.defaultCommand = DualStickArcadeDrive(drivetrain, primaryController)
        turnToAngleOnDpad(primaryController)
        JoystickButton(primaryController, kA.value).whenHeld(runGateForward())
        JoystickButton(primaryController, kB.value).whenHeld(runGateBackward())
        JoystickButton(primaryController, kX.value).whenHeld(runIndexerForward())
        JoystickButton(primaryController, kY.value).whenHeld(runIndexerBackward())
        Trigger({ primaryController.leftTriggerAxis > 0.2 }).whileActiveOnce(fixedSpeedShooter())
    }

    private fun configureAutoOptions() {
        // Drive forwards for 1.5s to clear tarmac [2pt]
    }


    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() = autoCommandChooser.selected
}