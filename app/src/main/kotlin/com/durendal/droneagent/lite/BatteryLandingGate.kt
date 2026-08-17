package com.durendal.droneagent.lite

/** Emits one forced-landing request per airborne low-battery event. */
internal class BatteryLandingGate {
    private var triggered = false

    fun evaluate(
        percent: Int?,
        flying: Boolean,
        connected: Boolean,
        landingRequested: Boolean,
    ): Boolean {
        if (!flying) {
            triggered = false
            return false
        }
        if (
            !connected || triggered || landingRequested ||
            percent == null || percent > FORCE_LANDING_PERCENT
        ) {
            return false
        }
        triggered = true
        return true
    }

    internal companion object {
        const val FORCE_LANDING_PERCENT = 13
    }
}
