// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance


private data class ConstantsListener(val func: () -> Unit, val id: Int)

class Constants {
    companion object {
        // ask edward: does allowing all constants to be modified via network tables put a penalty on performance/security/something else
        val constants = mutableMapOf(
            "kDrivetrainFrontLeftPort" to 0.0,
            "kDrivetrainFrontRightPort" to 1.0,
            "kDrivetrainBottomLeftPort" to 2.0,
            "kDrivetrainBottomRightPort" to 3.0
        )

        /** NetworkTables Constants Management **/
        private lateinit var table: NetworkTable
        private var listeners = mutableListOf<ConstantsListener>()
        private var listenersId = 0

        /**
         * Load constants from network tables
         */
        fun loadConstants() {
            table = NetworkTableInstance.getDefault().getTable("Constants")
            for (key in constants.keys) {
                if (constants[key] == null) continue

                val entry = table.getEntry(key)

                entry.setDefaultDouble(constants[key] ?: 0.0)
                entry.setPersistent()
                constants[key] = entry.getDouble(constants[key] ?: 0.0)

                entry.addListener({ event ->
                    onNetworkTablesChange()
                }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
            }
        }

        private fun onNetworkTablesChange() {
            /** update map **/
            for (key in constants.keys) {
                val entry = table.getEntry(key)
                constants[key] = entry.getDouble(constants[key] ?: 0.0)
            }
            listeners.map { listen -> listen.func() }
        }

        /**
         * Add a listener for any changes in the Preferences NetworkTable
         * listeners will be called on any change
         * Return an id for the listener that can latter be passed to removeListener
         */
        fun addListener(listener: () -> Unit): Int {
            val id = listenersId++
            listeners.add(ConstantsListener(listener, id))
            return id
        }

        /**
         * Remove a listener with an id created by a call to AddListener
         */
        fun removeListener(id: Int) {
            listeners.removeIf { listen -> listen.id == id }
        }
    }
}