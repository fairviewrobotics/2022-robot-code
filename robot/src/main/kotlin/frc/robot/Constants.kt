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

        val kDrivetrainFrontLeftPort get() = generateConstantGetter<Int>("kDrivetrainFrontLeftPort", 0.0)()
        val kDrivetrainFrontRightPort get() = generateConstantGetter<Int>("kDrivetrainFrontRightPort", 1.0)()
        val kDrivetrainBackLeftPort get() = generateConstantGetter<Int>("kDrivetrainBottomLeftPort", 2.0)()
        val kDrivetrainBackRightPort get() = generateConstantGetter<Int>("kDrivetrainBottomRightPort", 3.0)()

        val kDrivetrainLeftEncoderPortA get() = generateConstantGetter<Int>("kDrivetrainLeftEncoderPortA", 4.0)()
        val kDrivetrainLeftEncoderPortB get() = generateConstantGetter<Int>("kDrivetrainLeftEncoderPortB", 5.0)()
        val kDrivetrainRightEncoderPortA get() = generateConstantGetter<Int>("kDrivetrainRightEncoderPortA", 6.0)()
        val kDrivetrainRightEncoderPortB get() = generateConstantGetter<Int>("kDrivetrainRightEncoderPortB", 7.0)()

        val kDrivetrainEncoderAReversed = true
        val kDrivetrainEncoderBReversed = true

        val kDrivetrainPidP get() = generateConstantGetter<Double>("kDrivetrainPidP", 1.0)()
        val kDrivetrainPidI get() = generateConstantGetter<Double>("kDrivetrainPidI", 0.0)()
        val kDrivetrainPidD get() = generateConstantGetter<Double>("kDrivetrainPidD", 0.0)()

        val kDrivetrainMaxVelocity get() = generateConstantGetter<Double>("kDrivetrainMaxVelocity", 5.0)()
        val kDrivetrainMaxAngularVelocity get() = generateConstantGetter<Double>("kDrivetrainMaxAngularVelocity", 1.0)()
        val kDrivetrainMaxAcceleration get() = generateConstantGetter<Double>("kDrivetrainMaxAcceleration", 1.0)()

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


        // constants for basic pneumatics

        // SET THESE PROPERLY!
        val compressorID get() = generateConstantGetter<int>("compressorID", -1)()
        val solenoidModuleID get() = generateConstantGetter<int>("solenoidModuleID", -1)()
        val solenoidFChan get() = generateConstantGetter<int>("solenoidFChan", -1)()
        val solenoidRChan get() = generateConstantGetter<int>("solenoidRChan", -1)()

        // constants for flywheel LQR
        val shooterInertia get() = generateConstantGetter<Double>("shooterInertia", 0.0020521)() //  units: kg / m^2
        val shooterGearing get() = generateConstantGetter<Double>("shooterGearing", 1.0)() // output over input, unitless
        val shooterStateStdev get() = generateConstantGetter<Double>("shooterStateStdev", 3.0)()
        val shooterEncStdev get() = generateConstantGetter<Double>("shooterEncStdev", 0.01)()
        val shooterQ get() = generateConstantGetter<Double>("shooterQ", 8.0)()
        val shooterR get() = generateConstantGetter<Double>("shooterR", 12.0)()
        val shooterVolts get() =generateConstantGetter<Double>("shooterVolts", 12.0)()

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