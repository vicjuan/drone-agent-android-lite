package com.durendal.droneagent.lite

/** Repeating path that alternates a forward leg with a right half-turn. */
internal enum class ShuttleStep(val label: String) {
    FORWARD("前進"),
    TURN_RIGHT("右旋 180°");

    val isForward: Boolean
        get() = this == FORWARD

    fun next(): ShuttleStep =
        when (this) {
            FORWARD -> TURN_RIGHT
            TURN_RIGHT -> FORWARD
        }
}
