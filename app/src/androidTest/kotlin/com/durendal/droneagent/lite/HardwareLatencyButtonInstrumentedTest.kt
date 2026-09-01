package com.durendal.droneagent.lite

import android.content.Intent
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class HardwareLatencyButtonInstrumentedTest {

    @Test
    fun hardwareLatencySwitchIsPresentInTheFlightDashboard() {
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val activity = instrumentation.startActivitySync(
            Intent(instrumentation.targetContext, MainActivity::class.java)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
        )

        try {
            var switch: TextView? = null
            instrumentation.runOnMainSync {
                switch = activity.window.decorView.findTextView(HARDWARE_LATENCY_SWITCH_LABEL)
            }
            assertNotNull("hardware latency switch must be visible in the dashboard hierarchy", switch)
        } finally {
            instrumentation.runOnMainSync(activity::finish)
        }
    }

    @Test
    fun feedbackCircleSwitchesArePresentInTheExperimentRow() {
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val activity = instrumentation.startActivitySync(
            Intent(instrumentation.targetContext, MainActivity::class.java)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
        )

        try {
            var skatingCircleSwitch: TextView? = null
            var racingCircleSwitch: TextView? = null
            var fasterPhaseLeadSwitch: TextView? = null
            instrumentation.runOnMainSync {
                skatingCircleSwitch =
                    activity.window.decorView.findTextView(FEEDBACK_SKATING_CIRCLE_SWITCH_LABEL)
                racingCircleSwitch =
                    activity.window.decorView.findTextView(FEEDBACK_RACING_CIRCLE_SWITCH_LABEL)
                fasterPhaseLeadSwitch =
                    activity.window.decorView.findTextView(FASTER_PHASE_LEAD_SWITCH_LABEL)
            }
            assertNotNull(
                "feedback skating-circle switch must be visible in the dashboard hierarchy",
                skatingCircleSwitch,
            )
            assertNotNull(
                "feedback racing-circle switch must be visible in the dashboard hierarchy",
                racingCircleSwitch,
            )
            assertNotNull(
                "faster phase-lead switch must be visible in the dashboard hierarchy",
                fasterPhaseLeadSwitch,
            )
        } finally {
            instrumentation.runOnMainSync(activity::finish)
        }
    }

    @Test
    fun virtualStickFrameRateSwitchDefaultsToExperimentAndTogglesToBaseline() {
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val activity = instrumentation.startActivitySync(
            Intent(instrumentation.targetContext, MainActivity::class.java)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
        )

        try {
            var experimentSwitch: TextView? = null
            var baselineSwitch: TextView? = null
            instrumentation.runOnMainSync {
                experimentSwitch =
                    activity.window.decorView.findTextView(VIRTUAL_STICK_EXPERIMENT_RATE_LABEL)
                experimentSwitch?.performClick()
                baselineSwitch =
                    activity.window.decorView.findTextView(VIRTUAL_STICK_BASELINE_RATE_LABEL)
            }
            assertNotNull("40 Hz experiment must be selected by default", experimentSwitch)
            assertNotNull("switch must return to the 20 Hz baseline", baselineSwitch)
        } finally {
            instrumentation.runOnMainSync(activity::finish)
        }
    }

    @Test
    fun lowSpeedLapSwitchIsAbsentFromTheExperimentRow() {
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val activity = instrumentation.startActivitySync(
            Intent(instrumentation.targetContext, MainActivity::class.java)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
        )

        try {
            var switch: TextView? = null
            instrumentation.runOnMainSync {
                switch = activity.window.decorView.findTextView(REMOVED_LOW_SPEED_SWITCH_LABEL)
            }
            assertNull("removed low-speed lap switch must not remain in the dashboard hierarchy", switch)
        } finally {
            instrumentation.runOnMainSync(activity::finish)
        }
    }

    private fun View.findTextView(expectedText: String): TextView? {
        if (this is TextView && text.toString() == expectedText) return this
        if (this !is ViewGroup) return null
        for (index in 0 until childCount) {
            getChildAt(index).findTextView(expectedText)?.let { return it }
        }
        return null
    }

    private companion object {
        const val HARDWARE_LATENCY_SWITCH_LABEL = "硬體延遲脈衝・0.50 m/s"
        const val FEEDBACK_SKATING_CIRCLE_SWITCH_LABEL = "回授圓・滑冰 Ø2.0m・7.5秒×3"
        const val FEEDBACK_RACING_CIRCLE_SWITCH_LABEL = "回授圓・賽車 Ø2.0m・7.5秒×3"
        const val FASTER_PHASE_LEAD_SWITCH_LABEL = "方案 B：更快・1.35 m/s・超前 16°"
        const val VIRTUAL_STICK_BASELINE_RATE_LABEL = "VS 發送：20 Hz（基準）"
        const val VIRTUAL_STICK_EXPERIMENT_RATE_LABEL = "VS 發送：40 Hz（提速）"
        const val REMOVED_LOW_SPEED_SWITCH_LABEL = "低速完整圈・0.50 m/s"
    }
}
