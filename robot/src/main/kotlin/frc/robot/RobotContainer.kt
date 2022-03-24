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
import edu.wpi.first.networktables.NetworkTableInstance

import com.revrobotics.*
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
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
    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1), true)

    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidID.first,Constants.climbSolenoidID.second) 
    val intakeSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolenoidID.first, Constants.intakeSolenoidID.second)
    val climbPneumatics = SolenoidSubsystem(climbSolenoid)
    val intakePneumatics = SolenoidSubsystem(intakeSolenoid)

    // shooter

    val shooterMotor1 = WPI_TalonFX(Constants.shooterLowID)
    val shooterMotor2  = WPI_TalonFX(Constants.shooterHighID)
    val shooter1 = TalonFXShooterSubsystem(shooterMotor1, 1.0)
    val shooter2 = TalonFXShooterSubsystem(shooterMotor2, 1.0)
    // intake / indexer / gate
    val intake = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))
    val indexer = BallMotorSubsystem(WPI_TalonSRX(Constants.indexerID))
    val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))

    // gate color sensor
    val colorSensor = ColorSensorV3(I2C.Port.kOnboard)

    // auto command chooser
    var autoCommandChooser: SendableChooser<Command> = SendableChooser()

    // camera stuff
    // TODO: right place?
    val ntInst = NetworkTableInstance.getDefault()
    val cameraSelection = ntInst.getTable("").getEntry("CameraSelection") // todo: fill table name

    init {
        configureButtonBindings()
        configureAutoOptions()
    }

    /**
     * Controller ([GenericHID], [XboxController]) mapping.
     */
    private fun configureButtonBindings() {
        // EVERY OTHER CONTROL SCHEME IS DISHONEST.

        climbPneumatics.set(DoubleSolenoid.Value.kReverse)
        intakePneumatics.set(DoubleSolenoid.Value.kReverse)

        // Commands
        val shootVisually = { controller: XboxController -> ShootVision(drivetrain, shooter1, shooter2, gate, indexer, controller) }
        val autoClimb = { AutoClimb(winch, climbPneumatics) }
        
        val runGateForward = { FixedBallMotorSpeed(gate, { Constants.gateSpeed}) }
        val runGateBackward = { FixedBallMotorSpeed(gate, { -Constants.gateSpeed}) }
        val runIntakeForward = { FixedBallMotorSpeed(intake, { Constants.intakeSpeed}) }
        val runIntakeBackward = { FixedBallMotorSpeed(intake, { -Constants.intakeSpeed}) }
        val runIndexerForward = { FixedBallMotorSpeed(indexer, { -Constants.indexerSpeed}) }
        val runIndexerBackward = { FixedBallMotorSpeed(indexer, { -Constants.indexerSpeed}) }

        val runWinchDown = { controller: XboxController -> FixedWinchVoltage(winch, { -3.0 }) }
        val runWinchUp = { controller: XboxController -> FixedWinchVoltage(winch, { 3.0 }) }
        val runWinchAllTheWayUp = { WinchPIDCommand(winch, { Constants.elevatorMaxPos }) }
        val runWinchAllTheWayDown = { WinchPIDCommand(winch, { Constants.elevatorMinPos }) }
        val runWinchHalfway = { WinchPIDCommand(winch, { Constants.elevatorMaxPos * 0.5 }) }
        
        val setIntakePnemuaticUp = { PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kReverse) }
        val setIntakePnemuaticDown = { PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward) }
        val setClimberPneumaticForward = { PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kForward) }
        val setClimberPneumaticBackward = { PneumaticCommand(climbPneumatics, DoubleSolenoid.Value.kReverse) }
        
        val runIntakeIndexerGateUntilColorSensor = {
            ParallelCommandGroup(
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed} ),
                FixedBallMotorSpeed(indexer, { Constants.indexerSpeed }),
                GateSensored(gate, { Constants.gateSpeed }, colorSensor)
            )
        }

        val visionLineupToBall = { controller: XboxController ->
            ParallelCommandGroup(
                TurnToBall(controller, drivetrain),
                FixedBallMotorSpeed(intake, { Constants.intakeSpeed } )
            )
        }

        val visionLineupToHighGoal = { controller: XboxController ->
            SequentialCommandGroup(
                CheckVisionOrRumble(controller),
                TurnToHighGoal(drivetrain)
            )
        }

        val setManualShootingPower = { controller: XboxController ->
            ParallelCommandGroup(
                FixedShooterSpeed(shooter1, { controller.rightTriggerAxis }),
                FixedShooterSpeed(shooter2, { controller.rightTriggerAxis })
            )
        }

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
        
        val reverseIntakeIndexerGate = {
            ParallelCommandGroup(
                runIntakeBackward(),
                runIndexerBackward(),
                runGateBackward()
            )
        }
        
        // PRIMARY DRIVER

        // Joysticks - Arcade drive
        // LT - Vision Lineup to Ball
        // LB - Vision Lineup to High Goal
        // RT - Set Manual Shooting Power
        // RB - Visual Shooting
        // A - Intake Pneumatics Down
        // Y - Intake Pneumatics Up
        // B - Run Intake
        // X - Run Gate
        // D-Pad - Direct Angle Turn
        drivetrain.defaultCommand = DualStickArcadeDrive(drivetrain, primaryController)
        Trigger { primaryController.leftTriggerAxis > 0.2 }.whileActiveOnce ( visionLineupToBall(primaryController) )
        JoystickButton(primaryController, kLeftBumper.value).whenHeld(visionLineupToHighGoal(primaryController))
        Trigger { primaryController.rightTriggerAxis > 0.2 }.whileActiveOnce(fixedSpeedShooter())
        JoystickButton(primaryController, kRightBumper.value).whenHeld(shootVisually(primaryController))

        JoystickButton(primaryController, kA.value).whenHeld(setIntakePnemuaticUp())
        JoystickButton(primaryController, kB.value).whenHeld(setIntakePnemuaticDown())

        JoystickButton(primaryController, kX.value).whenHeld(runGateForward())
        JoystickButton(primaryController, kY.value).whenHeld(runIntakeForward())

        turnToAngleOnDpad(primaryController)

        // SECONDARY DRIVER

        // LT - Manually run climber up
        // RT - Manually run climber down
        // LB - Auto Climb
        // RB - Run Intake+Indexer+Gate Until Color
        // A - Climber Pneumatics Forward
        // B - Reverse Intake+Indexer+Gate
        // X - Climber Pneumatics Backward
        // Y - Direct Shooter

        Trigger({ secondaryController.rightTriggerAxis > 0.2 }).whileActiveOnce(runIntakeIndexerGateUntilColorSensor())
        JoystickButton(secondaryController, kRightBumper.value).whenHeld(reverseIntakeIndexerGate())

        Trigger({ secondaryController.leftTriggerAxis > 0.2 }).whileActiveOnce(runWinchUp(secondaryController))
        JoystickButton(secondaryController, kLeftBumper.value).whenHeld(runWinchDown(secondaryController))

        //POVButton(secondaryController, 0).whenHeld(runWinchAllTheWayUp())
        //POVButton(secondaryController, 180).whenHeld(runWinchAllTheWayDown())
        //POVButton(secondaryController, 135).whenHeld(runWinchHalfway())

        JoystickButton(secondaryController, kY.value).whenHeld(setIntakePnemuaticUp())
        JoystickButton(secondaryController, kA.value).whenHeld(setIntakePnemuaticDown())
        JoystickButton(secondaryController, kB.value).whenHeld(runGateBackward())

        // POVButton(secondaryController, 0).whenHeld(runWinchAllTheWayUp())
        // POVButton(secondaryController, 180).whenHeld(runWinchAllTheWayDown())
        // POVButton(secondaryController, 135).whenHeld(runWinchHalfway())

        JoystickButton(secondaryController, kLeftStick.value).whenHeld(AutoClimb(winch, climbPneumatics))

        POVButton(secondaryController, 0).whenHeld(setClimberPneumaticForward())
        POVButton(secondaryController, 180).whenHeld(setClimberPneumaticBackward())
        
        
        if (POVButton(secondaryController, 90).get()) {
            cameraSelection.setString(Constants.highGoalName)
        }
        if (POVButton(secondaryController, 270).get()){
            cameraSelection.setString(Constants.ballVisionName)
        }
        


    }

    private fun configureAutoOptions() {
        // Drive forwards for 1.5s to clear tarmac [2pt]
        autoCommandChooser.addOption("Drive forward [2pt]",
            DrivetrainPIDCommand(drivetrain) {
                DifferentialDriveWheelSpeeds(Constants.kDrivetrainFineForwardSpeed, Constants.kDrivetrainFineForwardSpeed)
            }.withTimeout(1.5)
        )
        // Shoot based on vision [4pt] then drive backwards to clear tarmac [2pt]
        autoCommandChooser.addOption("Shoot Vision + Drive forward [6pt]",
            SequentialCommandGroup(
                // shoot based on vision
                ShootVision(drivetrain, shooter1, shooter2, gate, indexer, primaryController, true).withTimeout(8.0),
                // shoot at default distance (just in case vision did not work)
                ShootDefaultDistance(shooter1, shooter2, gate, indexer, true).withTimeout(3.0),
                // drive off tarmac
                DrivetrainPIDCommand(drivetrain) {
                    DifferentialDriveWheelSpeeds(Constants.kDrivetrainFineForwardSpeed, Constants.kDrivetrainFineForwardSpeed)
                }.withTimeout(1.5)
            )
        )
    }


    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() = autoCommandChooser.selected
}