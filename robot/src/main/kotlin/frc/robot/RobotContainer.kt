// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.*
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.util.net.PortForwarder
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.POVButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.*
import frc.robot.subsystems.*
import java.lang.Math.PI

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
  // Controllers
  val primaryController = XboxController(0)
  val secondaryController = XboxController(1)

  // Motors, Solenoids, and Hardware Defintiions
  val motorFrontLeft =
      CANSparkMax(Constants.driveFrontLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
  val motorBackLeft =
      CANSparkMax(Constants.driveBackLeftID, CANSparkMaxLowLevel.MotorType.kBrushless)
  val motorFrontRight =
      CANSparkMax(Constants.driveFrontRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
  val motorBackRight =
      CANSparkMax(Constants.driveBackRightID, CANSparkMaxLowLevel.MotorType.kBrushless)
  val intakeSolenoid =
      DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.intakeSolenoidID.first,
          Constants.intakeSolenoidID.second
      )
  val lowShooterMotor = WPI_TalonFX(Constants.shooterLowID)
  val highShooterMotor = WPI_TalonFX(Constants.shooterHighID)

  // Subsytem Definitions
  val drivetrain =
      CANSparkMaxDrivetrainSubsystem(
          motorFrontLeft,
          motorBackLeft,
          motorFrontRight,
          motorBackRight,
          AHRS()
      )
  val intakePneumatics = SolenoidSubsystem(intakeSolenoid)
  val lowShooter = TalonFXShooterSubsystem(lowShooterMotor, 1.0)
  val highShooter = TalonFXShooterSubsystem(highShooterMotor, 1.0)
  val gate = BallMotorSubsystem(WPI_TalonSRX(Constants.gateID))
  val intakeIndexer = BallMotorSubsystem(WPI_TalonSRX(Constants.intakeID))

  var autoCommandChooser: SendableChooser<Command> = SendableChooser()
  init {
    PortForwarder.add(5800, "photonvision.local", 5800)
    configureButtonBindings()
    configureAutoOptions()
  }

  /** Controller ([GenericHID], [XboxController]) mapping. */
  private fun configureButtonBindings() {
    val runGate = { forward: Boolean ->
      if (forward) {
        FixedBallMotorSpeed(gate) { Constants.gateSpeed }
      } else {
        FixedBallMotorSpeed(gate) { -Constants.gateSpeed }
      }
    }

    val runIntakeIndexer = { forward: Boolean ->
      if (forward) {
        ParallelCommandGroup(
            FixedBallMotorSpeed(intakeIndexer) { Constants.intakeIndexerSpeed },
            FixedBallMotorSpeed(gate) { Constants.gateSpeed * .3 }
        )
      } else {
        ParallelCommandGroup(
            FixedBallMotorSpeed(intakeIndexer) { -Constants.intakeIndexerSpeed },
            FixedBallMotorSpeed(gate) { -Constants.gateSpeed * .3 }
        )
      }
    }

    val setIntakePneumatics = { forward: Boolean ->
      if (forward) {
        PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward)
      } else {
        PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kReverse)
      }
    }

    val primeShooter = { ->
      DualShooterPID(lowShooter, highShooter) {
        DualShootSpeed(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS)
      }
    }


    val turnToAngleOnDpad = { controller: XboxController ->
      // D-Pad: Turn to fixed angle
      for (i in 0 until 8) {
        val angleDeg = 45 * i
        POVButton(controller, angleDeg).whenHeld(TurnToAngle(drivetrain, { angleDeg * PI / 180.0 }))
      }
    }

    val fixedSpeedShooter = { speed1: Double, speed2: Double ->
      ParallelCommandGroup(
          DualShooterPID(lowShooter, highShooter) { DualShootSpeed(speed1, speed2) },
          ShootBallMotor(lowShooter, highShooter, gate, intakeIndexer),
          MaintainAngle(drivetrain)
      )
    }

    drivetrain.defaultCommand = DualStickArcadeDrive(drivetrain, primaryController)

    turnToAngleOnDpad(primaryController)

    JoystickButton(primaryController, kA.value).whenHeld(runIntakeIndexer(false))
    JoystickButton(primaryController, kB.value).whenHeld(runIntakeIndexer(true))
    JoystickButton(primaryController, kX.value).whenHeld(setIntakePneumatics(true))
    JoystickButton(primaryController, kY.value).whenHeld(setIntakePneumatics(false))

    JoystickButton(secondaryController, kA.value).whenHeld(runIntakeIndexer(true))
    JoystickButton(secondaryController, kB.value).whenHeld(runIntakeIndexer(false))
    JoystickButton(secondaryController, kX.value).whenHeld(setIntakePneumatics(true))
    JoystickButton(secondaryController, kY.value).whenHeld(setIntakePneumatics(false))

    Trigger({ primaryController.leftTriggerAxis > 0.2 })
        .whileActiveOnce(fixedSpeedShooter(350.0, -150.0)) // dumb
    Trigger({ primaryController.rightTriggerAxis > 0.2 })
        .whileActiveOnce(
            fixedSpeedShooter(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS)
        ) // fixed speed shoot
    Trigger({ primaryController.rightBumper }).whileActiveOnce(TurnToHighGoal(drivetrain))
    // Trigger({ primaryController.rightBumper }).whileActiveOnce(ShootVision(drivetrain,
    // lowShooter, highShooter, gate, intakeIndexer, primaryController))
    Trigger({ secondaryController.rightTriggerAxis > 0.2 }).whileActiveOnce(primeShooter())

    Trigger({ secondaryController.rightBumper })
        .whileActiveOnce(ShootOutake(lowShooter, highShooter, gate, intakeIndexer))
    Trigger({ secondaryController.leftBumper }).whileActiveOnce(runGate(false))
  }

  private fun configureAutoOptions() {

    val shootBall = { speed1: Double, speed2: Double ->
      ParallelCommandGroup(
          DualShooterPID(lowShooter, highShooter) { DualShootSpeed(speed1, speed2) },
          ShootBallMotor(lowShooter, highShooter, gate, intakeIndexer),
          MaintainAngle(drivetrain)
      )
    }
    val primeShooter = { ->
      DualShooterPID(lowShooter, highShooter) {
        DualShootSpeed(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS)
      }
    }
    // Drive forwards for 1.5s to clear tarmac [2pt]
    autoCommandChooser.addOption(
        "Drive forward [2pt]",
        DrivetrainPIDCommand(drivetrain) {
              DifferentialDriveWheelSpeeds(
                  Constants.kDrivetrainFineForwardSpeed,
                  Constants.kDrivetrainFineForwardSpeed
              )
            }
            .withTimeout(1.5)
    )
    // Shoot based on vision [4pt] then drive backwards to clear tarmac [2pt]
    autoCommandChooser.addOption(
        "Shoot Vision + Drive forward [6pt]",
        SequentialCommandGroup(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward).withTimeout(0.1),
            ParallelCommandGroup(
                    DrivetrainPIDCommand(drivetrain) {
                      DifferentialDriveWheelSpeeds(
                          -2.0 * Constants.kDrivetrainFineForwardSpeed,
                          -2.0 * Constants.kDrivetrainFineForwardSpeed
                      )
                    },
                    ParallelCommandGroup(
                        FixedBallMotorSpeed(intakeIndexer, { Constants.intakeIndexerSpeed }),
                        // GateSensored(gate, { Constants.gateSpeed }, colorSensor)
                        )
                )
                .withTimeout(2.0),
            DrivetrainPIDCommand(drivetrain) {
                  DifferentialDriveWheelSpeeds(
                      2.0 * Constants.kDrivetrainFineForwardSpeed,
                      2.0 * Constants.kDrivetrainFineForwardSpeed
                  )
                }
                .withTimeout(1.0),
            ParallelCommandGroup(
                    // TurnToHighGoal(drivetrain),
                    // ShooterSpinUpVision(lowShooter, highShooter)
                    )
                .withTimeout(3.0),
            ParallelCommandGroup(
                    ShootBallMotor(lowShooter, highShooter, gate, intakeIndexer),
                    MaintainAngle(drivetrain),
                    primeShooter()
                )
                .withTimeout(5.0),
            // shoot at default distance (just in case vision did not work)
            shootBall(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS).withTimeout(4.0),
        )
    )

    autoCommandChooser.setDefaultOption(
        "Static shoot + backup + vision shoot [10pt]",
        SequentialCommandGroup(
            PneumaticCommand(intakePneumatics, DoubleSolenoid.Value.kForward).withTimeout(0.1),
            DrivetrainPIDCommand(drivetrain) {
                  DifferentialDriveWheelSpeeds(
                      -1.0 * Constants.kDrivetrainFineForwardSpeed,
                      -1.0 * Constants.kDrivetrainFineForwardSpeed
                  )
                }
                .withTimeout(1.0),
            ShootBallMotor(lowShooter, highShooter, gate, intakeIndexer, true)
                .raceWith(
                    ParallelCommandGroup(
                        MaintainAngle(drivetrain),
                        DualShooterPID(lowShooter, highShooter, { DualShootSpeed(350.0, -100.0) })
                    )
                )
                .withTimeout(5.0),
            ParallelCommandGroup(
                    DrivetrainPIDCommand(drivetrain) {
                      DifferentialDriveWheelSpeeds(
                          -2.0 * Constants.kDrivetrainFineForwardSpeed,
                          -2.0 * Constants.kDrivetrainFineForwardSpeed
                      )
                    },
                    ParallelCommandGroup(
                        FixedBallMotorSpeed(intakeIndexer, { Constants.intakeIndexerSpeed }),
                        // GateSensored(gate, { Constants.gateSpeed }, colorSensor)
                        )
                )
                .withTimeout(1.5),
            DrivetrainPIDCommand(drivetrain) {
                  DifferentialDriveWheelSpeeds(
                      2.0 * Constants.kDrivetrainFineForwardSpeed,
                      2.0 * Constants.kDrivetrainFineForwardSpeed
                  )
                }
                .withTimeout(1.0),
            ParallelCommandGroup(
                    // TurnToHighGoal(drivetrain),
                    primeShooter()
                )
                .withTimeout(2.0),
            shootBall(Constants.shooterRadPerS, Constants.shooterAdjustRadPerS).withTimeout(5.0)
        )
    )

    SmartDashboard.putData("Auto Mode", autoCommandChooser)
  }
  val autonomousCommand: Command
    get() = autoCommandChooser.selected
}

