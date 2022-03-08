// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance

private data class ConstantsListener(val func: (newValue: Double) -> Unit, val id: Int)

class Constants {
    companion object {
        /* Port IDs */
        // drivetrain (4 motors, 2 on each side)
        val driveFrontLeftID = 1
        val driveBackLeftID = 2
        val driveFrontRightID = 3
        val driveBackRightID = 4

        // climber (2 solenoids and one winch motor)
        val climbSolenoidLeftID = Pair(1, 2)
        val climbSolenoidRightID = Pair(3, 4)
        val climbWinchID = 5

        // shooter (2 spark max to run wheels)
        val shooterLowID = 16
        val shooterHighID = 17

        // intake / indexer / gate (on talon each)
        val intakeID = 9
        val indexerID = 7
        val gateID = 8

        val refreshInterval = 0.02

        private val constants = mutableMapOf<String, Double>()

        /** NetworkTables Constants Management **/
        private val table: NetworkTable = NetworkTableInstance.getDefault().getTable("Constants")
        private var listeners = mutableMapOf<String, MutableList<ConstantsListener>>()
        private var listenersId = 0

        private fun<T> generateConstantGetter(key: String, default: Double): () -> T {
            val entry = table.getEntry(key)

            entry.setDefaultDouble(default)
            entry.setPersistent()
            constants[key] = entry.getDouble(default)

            entry.addListener({ event ->
                onNetworkTablesChange(event.name, event.value.value as Double)
            }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)

            return fun(): T {
                return (constants[key] ?: default) as T
            }
        }

        // drivetrain pid control coefficients
        val kDrivetrainPidP get() = generateConstantGetter<Double>("kDrivetrainPidP", 6.2296)()
        val kDrivetrainPidI get() = generateConstantGetter<Double>("kDrivetrainPidI", 0.0)()
        val kDrivetrainPidD get() = generateConstantGetter<Double>("kDrivetrainPidD", 0.0)()

        val kDrivetrainMaxVelocity get() = generateConstantGetter<Double>("kDrivetrainMaxVelocity", 2.6)()
        val kDrivetrainMaxAngularVelocity get() = generateConstantGetter<Double>("kDrivetrainMaxAngularVelocity", 10.0)()
        val kDrivetrainMaxAcceleration get() = generateConstantGetter<Double>("kDrivetrainMaxAcceleration", 30.0)()

        /* Shooter Feed-Forward gains. These gains control the open part (not feedback) of shooter control */
        // baseline (static) gain [V]
        val shooterFFS get() = generateConstantGetter<Double>("shooterFeedForwardS", 0.0)()
        // feedforward velocity gain [V / (rad/s)]
        // this should be approx: 12 V / (free speed in rad/s) = 1 / (kV in V/(rad/s))
        val shooterFFV get() = generateConstantGetter<Double>("shooterFeedForwardV", 0.002114165)()
        // feedforward acceleration gain [V / (rad/s^2)]
        // this should be approx: 12 V * (moment of inertia) /  (stall torque in Nm)
        val shooterFFA get() = generateConstantGetter<Double>("shooterFeedForwardA", 0.009757685)()

        val shooterP get() = generateConstantGetter<Double>("shooterP", 0.5)()
        val shooterI get() = generateConstantGetter<Double>("shooterI", 0.0)()
        val shooterD get() = generateConstantGetter<Double>("shooterD", 0.0)()

        val shooterElevationP get() = generateConstantGetter<Double>("shooterElevationP", 0.5)()
        val shooterElevationI get() = generateConstantGetter<Double>("shooterElevationI", 0.0)()
        val shooterElevationD get() = generateConstantGetter<Double>("shooterElevationD", 0.0)()

        val shooterElevationPosTolerance get() = generateConstantGetter<Double>("shooterElevationPosTolerance", 0.0)()
        val shooterElevationVelocityTolerance get() = generateConstantGetter<Double>("shooterElevationVelocityTolerance", 0.0)()

        // shooter target rpm
        val shooterRPM get() = generateConstantGetter<Double>("shooterRPM", 5000.0)()

        // constants for flywheel LQR
        val shooterInertia get() = 0.0020521 //  units: kg / m^2

        // base top speeds for intake, indexer, and gate
        val intakeSpeed = 0.5
        val indexerSpeed = 0.5
        val gateSpeed = 0.5


        private fun onNetworkTablesChange(key: String, value: Double) {
            /** update map **/
            constants[key] = value
            listeners[key]?.map { listen -> listen.func(value) }
        }

        /**
         * Add a listener for any changes in the Preferences NetworkTable
         * listeners will be called on any change
         * Return an id for the listener that can latter be passed to removeListener
         */
        fun addListener(key: String, listener: (value: Double) -> Unit): Int {
            val id = listenersId++
            if(listeners[key] == null) {
                listeners[key] = mutableListOf()
            }
            listeners[key]?.add(ConstantsListener(listener, id))
            return id
        }

        /**
         * Remove a listener with an id created by a call to AddListener
         */
        fun removeListener(key: String, id: Int) {
            listeners[key]?.removeIf { listen -> listen.id == id }
        }
    }
}