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
    fun experimentSwitchesArePresentInTheExperimentRow() {
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val activity = instrumentation.startActivitySync(
            Intent(instrumentation.targetContext, MainActivity::class.java)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
        )

        try {
            var skatingCircleSwitch: TextView? = null
            var racingCircleSwitch: TextView? = null
            var fasterPhaseLeadSwitch: TextView? = null
            var curvatureFeedforwardSwitch: TextView? = null
            var fastCruiseSwitch: TextView? = null
            var schemeCSwitch: TextView? = null
            var directionalVelocitySwitch: TextView? = null
            var fixedDirectionSpeedSwitch: TextView? = null
            instrumentation.runOnMainSync {
                skatingCircleSwitch =
                    activity.window.decorView.findTextView(MATHEMATICAL_SKATING_CIRCLE_SWITCH_LABEL)
                racingCircleSwitch =
                    activity.window.decorView.findTextView(MATHEMATICAL_RACING_CIRCLE_SWITCH_LABEL)
                fasterPhaseLeadSwitch =
                    activity.window.decorView.findTextView(FASTER_PHASE_LEAD_SWITCH_LABEL)
                curvatureFeedforwardSwitch =
                    activity.window.decorView.findTextView(CURVATURE_FEEDFORWARD_SWITCH_LABEL)
                fastCruiseSwitch =
                    activity.window.decorView.findTextView(FAST_CRUISE_SWITCH_LABEL)
                schemeCSwitch =
                    activity.window.decorView.findTextView(SCHEME_C_SWITCH_LABEL)
                directionalVelocitySwitch =
                    activity.window.decorView.findTextView(DIRECTIONAL_VELOCITY_SWITCH_LABEL)
                fixedDirectionSpeedSwitch =
                    activity.window.decorView.findTextView(FIXED_DIRECTION_SPEED_SWITCH_LABEL)
            }
            assertNotNull(
                "mathematical skating-circle switch must be visible in the dashboard hierarchy",
                skatingCircleSwitch,
            )
            assertNotNull(
                "mathematical racing-circle switch must be visible in the dashboard hierarchy",
                racingCircleSwitch,
            )
            assertNotNull(
                "faster phase-lead switch must be visible in the dashboard hierarchy",
                fasterPhaseLeadSwitch,
            )
            assertNotNull(
                "curvature-feedforward scheme B2 switch must be visible in the experiment row",
                curvatureFeedforwardSwitch,
            )
            assertNotNull(
                "scheme B3 switch must be visible in the experiment row",
                fastCruiseSwitch,
            )
            assertNotNull(
                "scheme C switch must expose its faster target speed in the dashboard hierarchy",
                schemeCSwitch,
            )
            assertNotNull(
                "four-direction velocity switch must be visible in the experiment row",
                directionalVelocitySwitch,
            )
            assertNotNull(
                "fixed-direction speed switch must be visible in the experiment row",
                fixedDirectionSpeedSwitch,
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
        const val MATHEMATICAL_SKATING_CIRCLE_SWITCH_LABEL = "數學圓・滑冰 Ø1.5m・7.5秒×3"
        const val MATHEMATICAL_RACING_CIRCLE_SWITCH_LABEL = "數學圓・賽車 Ø1.5m・6.75秒×3"
        const val FASTER_PHASE_LEAD_SWITCH_LABEL = "方案 B：更快・1.60 m/s・超前 16°"
        const val CURVATURE_FEEDFORWARD_SWITCH_LABEL =
            "方案 B2：曲率前饋・巡航 1.60 m/s・最大 1.90 m/s"
        const val FAST_CRUISE_SWITCH_LABEL =
            "方案 B3：曲率前饋・巡航 1.90 m/s・最大 2.00 m/s"
        const val SCHEME_C_SWITCH_LABEL = "方案 C：Ø1.5m 定曲率＋視覺修正・0.70 m/s"
        const val DIRECTIONAL_VELOCITY_SWITCH_LABEL = "四方向階躍：前・0.80 m/s・0.5 秒"
        const val FIXED_DIRECTION_SPEED_SWITCH_LABEL = "定向速度：前・漸進至 1.60 m/s"
        const val VIRTUAL_STICK_BASELINE_RATE_LABEL = "VS 發送：20 Hz（基準）"
        const val VIRTUAL_STICK_EXPERIMENT_RATE_LABEL = "VS 發送：40 Hz（提速）"
        const val REMOVED_LOW_SPEED_SWITCH_LABEL = "低速完整圈・0.50 m/s"
    }
}
