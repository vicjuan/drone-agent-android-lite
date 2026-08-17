package com.durendal.droneagent.lite

internal enum class TurnDirection(val commandSign: Double, val label: String) {
    LEFT(-1.0, "左旋"),
    RIGHT(1.0, "右旋"),
}

/** Tracks directed yaw progress across the -180/180-degree heading boundary. */
internal class HeadingTurn(
    val direction: TurnDirection,
    initialHeadingDegrees: Double,
) {
    private var directedDisplacementDegrees = 0.0
    private var previousHeadingDegrees = initialHeadingDegrees

    val progressDegrees: Double
        get() = directedDisplacementDegrees.coerceAtLeast(0.0)

    fun update(headingDegrees: Double): Double {
        val signedDelta = shortestAngularDelta(previousHeadingDegrees, headingDegrees)
        previousHeadingDegrees = headingDegrees
        directedDisplacementDegrees += signedDelta * direction.commandSign
        return (TARGET_DEGREES - progressDegrees).coerceAtLeast(0.0)
    }

    companion object {
        const val TARGET_DEGREES = 180.0

        internal fun shortestAngularDelta(fromDegrees: Double, toDegrees: Double): Double {
            var delta = (toDegrees - fromDegrees + 180.0) % 360.0
            if (delta < 0.0) delta += 360.0
            return delta - 180.0
        }
    }
}
