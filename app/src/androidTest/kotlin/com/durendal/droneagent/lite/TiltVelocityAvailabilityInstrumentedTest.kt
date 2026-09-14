package com.durendal.droneagent.lite

import android.content.Intent
import android.os.Build
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.v5.common.callback.CommonCallbacks
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.perception.data.ObstacleAvoidanceType
import dji.v5.manager.interfaces.IVirtualStickManager
import java.io.Closeable
import java.lang.reflect.Proxy
import java.util.Collections
import org.junit.Assert.*
import org.junit.Assume.assumeTrue
import org.junit.Test
import org.junit.runner.RunWith

/** Real Activity gates and outgoing frames; never run the SDK Activity on physical flight hardware. */
@RunWith(AndroidJUnit4::class)
class TiltVelocityAvailabilityInstrumentedTest {
    @Test
    fun unavailableVelocityDoesNotBlockStartingButKnownMotionAndAttitudeStillDo() {
        Harness().use { h ->
            for (state in listOf(VelocityState.MISSING, VelocityState.STALE, VelocityState.INVALID)) {
                h.velocity = state
                assertNull("$state incorrectly blocked starting", h.startFailure())
            }
            h.velocity = VelocityState.HOVER
            assertNull(h.startFailure())
            h.velocity = VelocityState.MOVING
            assertNotNull("Known motion must still prevent starting from an unsteady hover", h.startFailure())
            h.velocity = VelocityState.OVERSPEED
            assertNotNull(h.startFailure())
            h.velocity = VelocityState.MISSING
            assertNotNull("Missing velocity must not bypass attitude protection", h.startFailure(pitch = 16.0))
        }
    }

    @Test
    fun allTiltModesContinueWithoutVelocityAndAfterItsLossButStopOnFreshOverspeed() {
        Harness().use { h ->
            for (mode in TiltFlightMode.entries) {
                h.velocity = when (mode) {
                    TiltFlightMode.STRAIGHT -> VelocityState.MISSING
                    TiltFlightMode.CIRCLE -> VelocityState.STALE
                    TiltFlightMode.SKATING_CIRCLE -> VelocityState.INVALID
                }
                h.start(mode)
                h.waitUntil("$mode failed to enter its active tilt phase", 7_000) {
                    h.snapshot().any {
                        it.rollPitchControlMode == RollPitchControlMode.ANGLE &&
                            (mode == TiltFlightMode.STRAIGHT || it.roll < -3.0)
                    }
                }
                var count = h.angleCount()
                h.velocity = VelocityState.MOVING
                h.waitUntil("Normal measured motion stopped $mode after the settling phase", 700) {
                    h.angleCount() >= count + 4
                }
                count = h.angleCount()
                h.velocity = VelocityState.STALE
                h.waitUntil("Expired cached overspeed incorrectly stopped $mode", 700) {
                    h.angleCount() >= count + 4
                }
                val fastIndex = h.snapshot().size
                h.velocity = VelocityState.OVERSPEED
                h.waitUntil("Fresh overspeed did not promptly brake $mode", 750) {
                    h.snapshot().drop(fastIndex).any { it.rollPitchControlMode == RollPitchControlMode.VELOCITY }
                }
                h.waitUntil("$mode did not release control after braking", 5_000) { h.released() }
                val afterFast = h.snapshot().drop(fastIndex)
                val firstNeutral = afterFast.indexOfFirst { it.rollPitchControlMode == RollPitchControlMode.VELOCITY }
                assertTrue(afterFast.drop(firstNeutral).all {
                    it.rollPitchControlMode == RollPitchControlMode.VELOCITY && it.roll == 0.0 && it.pitch == 0.0
                })
                println("VELOCITY_GUARD mode=$mode unavailableAllowed=true lossAllowed=true freshOverspeedBraked=true")
            }
        }
    }

    private enum class VelocityState(val speed: Double) {
        MISSING(4.0), STALE(4.0), INVALID(Double.NaN), HOVER(0.1), MOVING(0.3), OVERSPEED(1.3),
    }

    private class Harness : Closeable {
        init {
            // The Activity also initializes non-stick DJI services. Keep those away from real aircraft.
            assumeTrue("Requires an isolated Android emulator", Build.HARDWARE in listOf("ranchu", "goldfish"))
        }
        val instrumentation = InstrumentationRegistry.getInstrumentation()
        val activity = instrumentation.startActivitySync(
            Intent(instrumentation.targetContext, MainActivity::class.java).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
        ) as MainActivity
        @Volatile var velocity = VelocityState.MISSING
        private val frames = Collections.synchronizedList(mutableListOf<VirtualStickFlightControlParam>())
        private val errors = Collections.synchronizedList(mutableListOf<Throwable>())
        private val runType = Class.forName("com.durendal.droneagent.lite.MainActivity\$TiltFlightRun")
        private val constructor = runType.getDeclaredConstructor(TiltFlightMode::class.java).apply { isAccessible = true }
        private val arm = MainActivity::class.java.getDeclaredMethod("armTiltFlight", runType).apply { isAccessible = true }
        private val prepare = MainActivity::class.java.getDeclaredMethod("driveTiltFlight", Long::class.javaPrimitiveType)
            .apply { isAccessible = true }
        private val record = MainActivity::class.java.getDeclaredMethod("recordVirtualStickFrame", VirtualStickFrameProfile::class.java)
            .apply { isAccessible = true }
        private val startGate = MainActivity::class.java.getDeclaredMethod("tiltFlightStartFailure", Long::class.javaPrimitiveType)
            .apply { isAccessible = true }
        private val manager = Proxy.newProxyInstance(
            IVirtualStickManager::class.java.classLoader, arrayOf(IVirtualStickManager::class.java),
        ) { _, method, args ->
            when (method.name) {
                "enableVirtualStick", "disableVirtualStick" ->
                    (args!![0] as CommonCallbacks.CompletionCallback).onSuccess()
                "sendVirtualStickAdvancedParam" -> frames.add(args!![0] as VirtualStickFlightControlParam)
            }
            null
        } as IVirtualStickManager
        private val session = VirtualStickSession(
            onStatus = {},
            onBeforeFrame = { now ->
                try {
                    synchronized(activity) {
                        val run = activity.field("tiltFlightRun")
                        val started = run?.field("startedAtNanos") as? Long ?: 0L
                        val heading = if (started == 0L) 0.0 else
                            (run!!.field("controller") as TiltFlightController).command(now, 0.0).yawHeadingDegrees
                        scene(now, heading)
                        prepare.invoke(activity, now)
                    }
                } catch (error: Throwable) { errors.add(error) }
            },
            onFrameSent = { profile ->
                try { record.invoke(activity, profile) } catch (error: Throwable) { errors.add(error) }
            },
            manager = manager,
        )
        init {
            instrumentation.runOnMainSync {
                KeyManager.getInstance().cancelListen(activity)
                (activity.field("virtualStick") as VirtualStickSession).close()
                activity.setField("virtualStick", session)
                session.selectFrameRate(VirtualStickFrameRate.BASELINE_20)
                scene(System.nanoTime(), 0.0)
                activity.setField("avoidance", AvoidanceCheck.Status(ObstacleAvoidanceType.BRAKE, true))
                activity.setField("stickStatus", VirtualStickStatus())
            }
        }
        fun startFailure(pitch: Double = 0.0): String? {
            var failure: String? = null
            instrumentation.runOnMainSync {
                val now = System.nanoTime()
                scene(now, 0.0)
                activity.setField("aircraftPitchDegrees", pitch)
                failure = startGate.invoke(activity, now) as String?
            }
            return failure
        }
        fun start(mode: TiltFlightMode) {
            frames.clear()
            instrumentation.runOnMainSync {
                scene(System.nanoTime(), 0.0)
                activity.setField("avoidance", AvoidanceCheck.Status(ObstacleAvoidanceType.CLOSE, true))
                activity.setField("stickOwned", true)
                activity.setField("stickStatus", VirtualStickStatus(true, true, "MSDK"))
                val run = constructor.newInstance(mode)
                activity.setField("tiltFlightRun", run)
                session.enable {}
                arm.invoke(activity, run)
            }
        }
        fun scene(now: Long, heading: Double) {
            activity.setField("registered", true)
            activity.setField("aircraftConnected", true)
            activity.setField("flying", true)
            activity.setField("activityForeground", true)
            activity.setField("altitudeMeters", 0.75)
            activity.setField("altitudeAtNanos", now)
            activity.setField("aircraftHeadingDegrees", heading)
            activity.setField("aircraftPitchDegrees", 0.0)
            activity.setField("aircraftRollDegrees", 0.0)
            activity.setField("aircraftHeadingAtNanos", now)
            val state = velocity
            synchronized(requireNotNull(activity.field("aircraftVelocityLock"))) {
                activity.setField("groundSpeedMetersPerSecond", state.speed)
                activity.setField("aircraftVelocityAtNanos", when (state) {
                    VelocityState.MISSING -> 0L
                    VelocityState.STALE -> now - 2_000_000_000L
                    else -> now
                })
            }
        }
        fun snapshot(): List<VirtualStickFlightControlParam> = synchronized(frames) { frames.toList() }
        fun angleCount(): Int = synchronized(frames) { frames.count { it.rollPitchControlMode == RollPitchControlMode.ANGLE } }
        fun released(): Boolean = activity.field("tiltFlightRun") == null &&
            activity.field("stickOwned") == false && activity.field("stickTransitionPending") == false
        fun waitUntil(message: String, timeoutMs: Long, condition: () -> Boolean) {
            val deadline = System.nanoTime() + timeoutMs * 1_000_000L
            while (!condition() && errors.isEmpty() && System.nanoTime() < deadline) Thread.sleep(10)
            assertTrue("Activity callback failed: $errors", errors.isEmpty())
            assertTrue(message, condition())
        }
        override fun close() {
            instrumentation.runOnMainSync {
                activity.setField("tiltFlightRun", null)
                activity.setField("flying", false)
                activity.setField("aircraftConnected", false)
                activity.setField("stickOwned", false)
                session.close()
                activity.finish()
            }
        }
    }
    companion object {
        private fun Any.field(name: String): Any? = javaClass.getDeclaredField(name).apply { isAccessible = true }.get(this)
        private fun Any.setField(name: String, value: Any?) = javaClass.getDeclaredField(name).apply { isAccessible = true }.set(this, value)
    }
}
