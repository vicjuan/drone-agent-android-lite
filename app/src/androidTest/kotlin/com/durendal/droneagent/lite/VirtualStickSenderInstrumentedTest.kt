package com.durendal.droneagent.lite

import androidx.test.ext.junit.runners.AndroidJUnit4
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthority
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.v5.common.callback.CommonCallbacks
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener
import dji.v5.manager.interfaces.IVirtualStickManager
import java.lang.reflect.Proxy
import java.util.Collections
import java.util.concurrent.CountDownLatch
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class VirtualStickSenderInstrumentedTest {
    @Test
    fun releaseWaitsForEarlierSubmissionBeforeSendingFinalNeutral() {
        val motionEntered = CountDownLatch(1)
        val finishMotion = CountDownLatch(1)
        val releaseStarted = CountDownLatch(1)
        val releaseCompleted = CountDownLatch(1)
        val submitted = Collections.synchronizedList(mutableListOf<Double>())
        val session = VirtualStickSession(onStatus = {}, manager = manager { param ->
            if (param.roll != 0.0) {
                motionEntered.countDown()
                check(finishMotion.await(5, TimeUnit.SECONDS))
            }
            submitted.add(param.roll)
        })
        var releaseThread: Thread? = null
        try {
            session.enable {}
            session.setHorizontalVelocity(0.3, 0.0, System.nanoTime() + 2_000_000_000L)
            assertTrue(motionEntered.await(2, TimeUnit.SECONDS))
            releaseThread = Thread {
                releaseStarted.countDown()
                session.disable { releaseCompleted.countDown() }
            }.apply { start() }
            assertTrue(releaseStarted.await(2, TimeUnit.SECONDS))
            assertFalse("release must not overtake an unfinished motion submission", releaseCompleted.await(200, TimeUnit.MILLISECONDS))
            finishMotion.countDown()
            assertTrue(releaseCompleted.await(2, TimeUnit.SECONDS))
            assertEquals(0.0, submitted.last(), 0.0)
        } finally {
            finishMotion.countDown()
            releaseThread?.join(2_000L)
            session.close()
        }
    }

    @Test
    fun expiredHorizontalCommandBecomesNeutralWithoutChangingYawOrClimb() {
        val motionSent = CountDownLatch(1)
        val neutralAfterMotion = CountDownLatch(1)
        val session = VirtualStickSession(onStatus = {}, manager = manager { param ->
            if (param.roll != 0.0) motionSent.countDown()
            if (motionSent.count == 0L && param.roll == 0.0) {
                assertEquals(0.12, param.verticalThrottle, 0.0)
                assertEquals(30.0, param.yaw, 0.0)
                neutralAfterMotion.countDown()
            }
        })
        try {
            session.enable {}
            session.setClimbRate(0.12)
            session.setYawHeading(30.0)
            session.setHorizontalVelocity(0.3, 0.0, System.nanoTime() + 300_000_000L)
            assertTrue(motionSent.await(2, TimeUnit.SECONDS))
            assertTrue(neutralAfterMotion.await(2, TimeUnit.SECONDS))
        } finally {
            session.close()
        }
    }

    @Test
    fun explicitStageTwoEnvelopeReachesSenderWithoutChangingLegacyCommands() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        val session = VirtualStickSession(onStatus = {}, manager = manager { submitted.add(it) })
        fun awaitAxes(forward: Double, right: Double) {
            val deadline = System.nanoTime() + 2_000_000_000L
            while (System.nanoTime() < deadline) {
                val param = submitted.poll(100, TimeUnit.MILLISECONDS) ?: continue
                if (
                    kotlin.math.abs(param.roll - forward) < 1e-9 &&
                    kotlin.math.abs(param.pitch - right) < 1e-9
                ) return
            }
            throw AssertionError("Sender did not produce ($forward, $right)")
        }
        try {
            session.enable {}
            session.setHorizontalVelocity(2.35, 0.0, maximumMagnitudeMetersPerSecond = 2.35)
            awaitAxes(2.35, 0.0)
            session.setHorizontalVelocity(3.0, 4.0, maximumMagnitudeMetersPerSecond = 2.35)
            awaitAxes(1.41, 1.88)
            // The opt-in is per command, not retained for a later experiment.
            session.setHorizontalVelocity(2.35, 0.0)
            awaitAxes(2.0, 0.0)
        } finally {
            session.close()
        }
    }

    @Test
    fun angleLeaseExpiresToVelocityZeroWithoutProducerAndKeepsProfileUnitsDistinct() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        val profiles = LinkedBlockingQueue<VirtualStickFrameProfile>()
        val session = VirtualStickSession(
            onStatus = {},
            onFrameSent = { profiles.add(it) },
            manager = manager { submitted.add(it) },
        )
        try {
            session.enable {}
            session.setClimbRate(0.12)
            session.setYawHeading(30.0)
            // A distant producer deadline must still be capped to the short ANGLE lease.
            assertTrue(session.setHorizontalAngles(1.0, -3.0, System.nanoTime() + 5_000_000_000L))
            val angle = awaitFrame(submitted) { it.rollPitchControlMode == RollPitchControlMode.ANGLE }
            assertEquals(1.0, angle.roll, 0.0)
            assertEquals(-3.0, angle.pitch, 0.0)
            val neutral = awaitFrame(submitted) { it.rollPitchControlMode == RollPitchControlMode.VELOCITY }
            assertEquals(0.0, neutral.roll, 0.0)
            assertEquals(0.0, neutral.pitch, 0.0)
            assertEquals(0.12, neutral.verticalThrottle, 0.0)
            assertEquals(30.0, neutral.yaw, 0.0)
            val profileDeadline = System.nanoTime() + 2_000_000_000L
            var angleProfile: VirtualStickFrameProfile? = null
            var neutralProfile: VirtualStickFrameProfile? = null
            while (System.nanoTime() < profileDeadline && neutralProfile == null) {
                val profile = profiles.poll(100, TimeUnit.MILLISECONDS) ?: continue
                if (profile.rollPitchMode == "ANGLE") angleProfile = profile
                if (angleProfile != null && profile.rollPitchMode == "VELOCITY") neutralProfile = profile
            }
            val tilted = requireNotNull(angleProfile)
            assertNull(tilted.forwardMetersPerSecond)
            assertNull(tilted.rightMetersPerSecond)
            assertEquals(1.0, requireNotNull(tilted.rollDegrees), 0.0)
            assertEquals(-3.0, requireNotNull(tilted.pitchDegrees), 0.0)
            val stopped = requireNotNull(neutralProfile)
            assertNull(stopped.rollDegrees)
            assertNull(stopped.pitchDegrees)
            assertEquals(0.0, requireNotNull(stopped.forwardMetersPerSecond), 0.0)
            assertEquals(0.0, requireNotNull(stopped.rightMetersPerSecond), 0.0)
        } finally {
            session.close()
        }
    }

    @Test
    fun rejectedAnglesClearPriorTiltRatherThanRetainingIt() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        val session = VirtualStickSession(onStatus = {}, manager = manager { submitted.add(it) })
        try {
            session.enable {}
            val invalidCommands = listOf<(Long) -> Triple<Double, Double, Long>>(
                { now -> Triple(Double.NaN, 0.0, now + 150_000_000L) },
                { now -> Triple(0.0, Double.POSITIVE_INFINITY, now + 150_000_000L) },
                { now -> Triple(4.0, -4.0, now + 150_000_000L) },
                { _ -> Triple(0.0, -3.0, 0L) },
                { now -> Triple(0.0, -3.0, now - 1L) },
            )
            for (invalidCommand in invalidCommands) {
                assertTrue(session.setHorizontalAngles(0.0, -3.0, System.nanoTime() + 150_000_000L))
                awaitFrame(submitted) { it.rollPitchControlMode == RollPitchControlMode.ANGLE }
                val (roll, pitch, deadline) = invalidCommand(System.nanoTime())
                assertFalse(session.setHorizontalAngles(roll, pitch, deadline))
                // Setter and submission share the monitor: nothing queued after this
                // clear can be an older command racing the rejected update.
                submitted.clear()
                val next = requireNotNull(submitted.poll(2, TimeUnit.SECONDS))
                assertEquals(RollPitchControlMode.VELOCITY, next.rollPitchControlMode)
                assertEquals(0.0, next.roll, 0.0)
                assertEquals(0.0, next.pitch, 0.0)
            }
        } finally {
            session.close()
        }
    }

    @Test
    fun legacyHorizontalSettersExitAngleModeWithTheirOwnVelocityEnvelopes() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        val session = VirtualStickSession(onStatus = {}, manager = manager { submitted.add(it) })
        fun replaceTilt(command: () -> Unit, forward: Double, right: Double) {
            assertTrue(session.setHorizontalAngles(1.0, -3.0, System.nanoTime() + 150_000_000L))
            awaitFrame(submitted) { it.rollPitchControlMode == RollPitchControlMode.ANGLE }
            command()
            submitted.clear()
            val next = requireNotNull(submitted.poll(2, TimeUnit.SECONDS))
            assertEquals(RollPitchControlMode.VELOCITY, next.rollPitchControlMode)
            assertEquals(forward, next.roll, 0.0)
            assertEquals(right, next.pitch, 0.0)
        }
        try {
            session.enable {}
            replaceTilt({ session.setHorizontalVelocity(3.0, -3.0) }, 2.0, -2.0)
            replaceTilt({ session.setForwardOnly(1.0) }, 0.5, 0.0)
            replaceTilt({ session.setStick(StickSide.RIGHT, -0.5, 1.0) }, 0.5, -0.25)
            replaceTilt({ session.setHorizontalVelocity(0.0, 0.0) }, 0.0, 0.0)
        } finally {
            session.close()
        }
    }

    @Test
    fun disableWaitsForAngleSubmissionThenSendsVelocityNeutralBeforeReleasing() {
        val angleEntered = CountDownLatch(1)
        val finishAngle = CountDownLatch(1)
        val releaseStarted = CountDownLatch(1)
        val releaseCompleted = CountDownLatch(1)
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        val lifecycle = Collections.synchronizedList(mutableListOf<String>())
        val session = VirtualStickSession(
            onStatus = {},
            manager = manager(
                onDisable = { lifecycle.add("disable") },
                onAdvancedMode = { enabled -> if (!enabled) lifecycle.add("advancedOff") },
            ) { param ->
                if (param.rollPitchControlMode == RollPitchControlMode.ANGLE) {
                    angleEntered.countDown()
                    check(finishAngle.await(5, TimeUnit.SECONDS))
                }
                submitted.add(param)
                lifecycle.add("frame")
            },
        )
        var releaseThread: Thread? = null
        try {
            session.enable {}
            assertTrue(session.setHorizontalAngles(1.0, -3.0, System.nanoTime() + 150_000_000L))
            assertTrue(angleEntered.await(2, TimeUnit.SECONDS))
            releaseThread = Thread {
                releaseStarted.countDown()
                session.disable { releaseCompleted.countDown() }
            }.apply { start() }
            assertTrue(releaseStarted.await(2, TimeUnit.SECONDS))
            assertFalse(releaseCompleted.await(200, TimeUnit.MILLISECONDS))
            finishAngle.countDown()
            assertTrue(releaseCompleted.await(2, TimeUnit.SECONDS))
            val finalFrame = submitted.last()
            assertEquals(RollPitchControlMode.VELOCITY, finalFrame.rollPitchControlMode)
            assertEquals(0.0, finalFrame.roll, 0.0)
            assertEquals(0.0, finalFrame.pitch, 0.0)
            assertEquals(0.0, finalFrame.verticalThrottle, 0.0)
            assertEquals(0.0, finalFrame.yaw, 0.0)
            assertEquals(listOf("frame", "advancedOff", "disable"), lifecycle.takeLast(3))
            assertFalse(session.setHorizontalAngles(0.0, -3.0, System.nanoTime() + 150_000_000L))
            submitted.clear()
            session.enable {}
            val resumed = requireNotNull(submitted.poll(2, TimeUnit.SECONDS))
            assertEquals(RollPitchControlMode.VELOCITY, resumed.rollPitchControlMode)
            assertEquals(0.0, resumed.roll, 0.0)
            assertEquals(0.0, resumed.pitch, 0.0)
        } finally {
            finishAngle.countDown()
            releaseThread?.join(2_000L)
            session.close()
        }
    }

    @Test
    fun adjacentTiltPhasesDoNotInsertAVelocityBrake() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        val controller = TiltFlightController(TiltFlightMode.CIRCLE)
        var phaseTime = controller.start(1L, 0.0).phaseEndsAtNanos
        lateinit var session: VirtualStickSession
        session = VirtualStickSession(
            onStatus = {},
            onBeforeFrame = { now ->
                val command = controller.command(phaseTime, 0.0)
                phaseTime = command.phaseEndsAtNanos
                if (command.usesAngle) {
                    check(session.setHorizontalAngles(
                        command.rollDegrees, command.pitchDegrees, now + 150_000_000L,
                    ))
                } else {
                    session.setHorizontalVelocity(0.0, 0.0)
                }
            },
            manager = manager { submitted.add(it) },
        )
        try {
            session.enable {}
            val startup = requireNotNull(submitted.poll(2, TimeUnit.SECONDS))
            val circle = requireNotNull(submitted.poll(2, TimeUnit.SECONDS))
            val brake = requireNotNull(submitted.poll(2, TimeUnit.SECONDS))
            assertEquals(RollPitchControlMode.ANGLE, startup.rollPitchControlMode)
            assertEquals(-3.0, startup.pitch, 0.0)
            assertEquals(RollPitchControlMode.ANGLE, circle.rollPitchControlMode)
            assertTrue(circle.roll < -3.0)
            assertEquals(RollPitchControlMode.VELOCITY, brake.rollPitchControlMode)
            assertEquals(0.0, brake.roll, 0.0)
            assertEquals(0.0, brake.pitch, 0.0)
        } finally {
            session.close()
        }
    }

    @Test
    fun unknownAuthorityKeepsSendingAcrossMsdkTransitions() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        lateinit var listener: VirtualStickStateListener
        var status = VirtualStickStatus()
        val session = VirtualStickSession(
            onStatus = { status = it },
            manager = manager(onStateListener = { listener = it }) { submitted.add(it) },
        )
        try {
            session.start()
            session.enable {}
            for ((owner, speed) in listOf(
                FlightControlAuthority.UNKNOWN to 0.3,
                FlightControlAuthority.MSDK to 0.4,
                FlightControlAuthority.UNKNOWN to 0.5,
            )) {
                listener.onVirtualStickStateUpdate(VirtualStickState(true, owner, true))
                session.setHorizontalVelocity(speed, 0.0)
                awaitFrame(submitted) { it.roll == speed }
                assertTrue("The experiment must accept $owner while enabled", status.hasMsdkAuthority)
                assertFalse("An active unknown owner must not confirm release", status.isReleased)
                assertEquals("Keep the original owner visible to diagnostics", owner.name, status.authority)
            }
        } finally {
            session.close()
        }
    }

    @Test
    fun explicitTakeoverAndDisabledUnknownStillStopTheSender() {
        val submitted = LinkedBlockingQueue<VirtualStickFlightControlParam>()
        lateinit var listener: VirtualStickStateListener
        var status = VirtualStickStatus()
        val session = VirtualStickSession(
            onStatus = { status = it },
            manager = manager(onStateListener = { listener = it }) { submitted.add(it) },
        )
        try {
            session.start()
            session.enable {}
            for ((enabled, owner) in listOf(
                true to FlightControlAuthority.RC,
                true to FlightControlAuthority.OSDK,
                false to FlightControlAuthority.UNKNOWN,
            )) {
                listener.onVirtualStickStateUpdate(VirtualStickState(true, FlightControlAuthority.UNKNOWN, true))
                session.setHorizontalVelocity(0.3, 0.0)
                awaitFrame(submitted) { it.roll == 0.3 }
                listener.onVirtualStickStateUpdate(VirtualStickState(enabled, owner, true))
                submitted.clear()
                assertNull("Sending must stop for enabled=$enabled owner=$owner",
                    submitted.poll(200, TimeUnit.MILLISECONDS))
                assertFalse(status.hasMsdkAuthority)
                assertEquals(owner != FlightControlAuthority.OSDK, status.isReleased)
            }
        } finally {
            session.close()
        }
    }

    private fun awaitFrame(
        submitted: LinkedBlockingQueue<VirtualStickFlightControlParam>,
        matches: (VirtualStickFlightControlParam) -> Boolean,
    ): VirtualStickFlightControlParam {
        val deadline = System.nanoTime() + 2_000_000_000L
        while (System.nanoTime() < deadline) {
            val param = submitted.poll(100, TimeUnit.MILLISECONDS) ?: continue
            if (matches(param)) return param
        }
        throw AssertionError("Sender did not produce the expected frame")
    }

    private fun manager(
        onDisable: () -> Unit = {},
        onAdvancedMode: (Boolean) -> Unit = {},
        onStateListener: (VirtualStickStateListener) -> Unit = {},
        submit: (VirtualStickFlightControlParam) -> Unit,
    ): IVirtualStickManager =
        Proxy.newProxyInstance(
            IVirtualStickManager::class.java.classLoader,
            arrayOf(IVirtualStickManager::class.java),
        ) { _, method, arguments ->
            when (method.name) {
                "setVirtualStickStateListener" -> onStateListener(arguments!![0] as VirtualStickStateListener)
                "disableVirtualStick" -> {
                    onDisable()
                    (arguments!![0] as CommonCallbacks.CompletionCallback).onSuccess()
                }
                "setVirtualStickAdvancedModeEnabled" -> onAdvancedMode(arguments!![0] as Boolean)
                "enableVirtualStick" ->
                    (arguments!![0] as CommonCallbacks.CompletionCallback).onSuccess()
                "sendVirtualStickAdvancedParam" -> submit(arguments!![0] as VirtualStickFlightControlParam)
            }
            null
        } as IVirtualStickManager
}
