package com.durendal.droneagent.lite

import java.util.Random
import androidx.test.ext.junit.runners.AndroidJUnit4
import java.util.concurrent.CountDownLatch
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.core.Point
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

@RunWith(AndroidJUnit4::class)
class VisualVelocityDiagnosticsInstrumentedTest {
    @Test
    fun restartingWhileOldResultIsInFlightDropsBusyFramesAndStartsWithAFreshBaseline() {
        val oldResultEntered = CountDownLatch(1)
        val releaseOldResult = CountDownLatch(1)
        val oldResultReturned = CountDownLatch(1)
        val results = LinkedBlockingQueue<Map<String, String>>()
        val diagnostic = VisualVelocityDiagnostics { event, _, details ->
            if (event == "visual_velocity_result") {
                val fields = details.split(' ').associate { it.substringBefore('=') to it.substringAfter('=') }
                if (fields["attemptId"] == "old") {
                    oldResultEntered.countDown()
                    try {
                        check(releaseOldResult.await(10, TimeUnit.SECONDS))
                    } finally {
                        oldResultReturned.countDown()
                    }
                }
                results.add(fields)
            }
        }
        val frame = Mat(180, 320, CvType.CV_8UC4, Scalar(180.0, 120.0, 50.0, 255.0))
        try {
            diagnostic.start("old", "ACQUIRING", 0.0)
            diagnostic.submitRgba(frame, System.nanoTime(), context())
            assertTrue("old result did not arrive", oldResultEntered.await(10, TimeUnit.SECONDS))
            diagnostic.stop()
            diagnostic.start("new", "FORWARD", 0.0)
            // Clear the admission-rate interval while deliberately retaining the old result.
            Thread.sleep(80)
            val busyFrameNanos = System.nanoTime()
            diagnostic.submitRgba(frame, busyFrameNanos, context())
            releaseOldResult.countDown()
            assertTrue(oldResultReturned.await(10, TimeUnit.SECONDS))
            assertEquals("old", results.poll(10, TimeUnit.SECONDS)?.get("attemptId"))

            // Retry after the old callback exits; no queued new-generation frame is allowed.
            var newResult: Map<String, String>? = null
            val deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(10)
            while (newResult == null && System.nanoTime() < deadline) {
                Thread.sleep(80)
                diagnostic.submitRgba(frame, System.nanoTime(), context())
                newResult = results.poll(200, TimeUnit.MILLISECONDS)
            }
            val result = checkNotNull(newResult) { "new attempt did not produce a result" }
            assertEquals("new", result["attemptId"])
            assertEquals("BASELINE", result["reason"])
            assertEquals("null", result["previousFrameNanos"])
            assertTrue(checkNotNull(result["frameNanos"]).toLong() > busyFrameNanos)
            assertTrue(checkNotNull(result["busyDropped"]).toLong() >= 1L)
        } finally {
            releaseOldResult.countDown()
            diagnostic.close()
            frame.release()
        }
    }

    @Test
    fun oldTrackingTeardownCannotStopOrRelabelAReplacementStraightSession() {
        val results = LinkedBlockingQueue<Map<String, String>>()
        val diagnostic = VisualVelocityDiagnostics { event, _, details ->
            if (event == "visual_velocity_result") {
                results.add(details.split(' ').associate { it.substringBefore('=') to it.substringAfter('=') })
            }
        }
        val frame = Mat(180, 320, CvType.CV_8UC4, Scalar(180.0, 120.0, 50.0, 255.0))
        try {
            diagnostic.start("b2", "RECENTERING", 0.0)
            diagnostic.start("straight", "ACQUIRING", 0.0)
            diagnostic.stop("b2")
            diagnostic.updatePhase("DISABLED", "b2")
            assertTrue(diagnostic.isActive())
            diagnostic.submitRgba(frame, System.nanoTime(), context())
            val result = checkNotNull(results.poll(10, TimeUnit.SECONDS))
            assertEquals("straight", result["attemptId"])
            assertEquals("ACQUIRING", result["phase"])
            assertEquals("true", result["scopeActive"])
            diagnostic.stop("straight")
            assertTrue(!diagnostic.isActive())
        } finally {
            diagnostic.close()
            frame.release()
        }
    }

    @Test
    fun unsupportedContextRetainsPixelMotionButCannotPublishCalibratedSpeed() {
        val results = LinkedBlockingQueue<Map<String, String>>()
        val diagnostic = VisualVelocityDiagnostics { event, _, details ->
            if (event == "visual_velocity_result") {
                results.add(details.split(' ').associate { it.substringBefore('=') to it.substringAfter('=') })
            }
        }
        val frame = calibratedFloor()
        try {
            diagnostic.start("b2", "TRACKING", 0.0)
            var calibrated: Map<String, String>? = null
            repeat(5) { calibrated = observe(diagnostic, frame, results) }
            // A measured marker scale must work without any altitude telemetry.
            assertEquals("true", calibrated?.get("valueValid"))
            assertEquals(0.0, checkNotNull(calibrated?.get("forwardMps")).toDouble(), 0.005)
            assertEquals(0.0, checkNotNull(calibrated?.get("rightMps")).toDouble(), 0.005)

            val tilted = observe(diagnostic, frame, results) {
                it.copy(cameraPitchCommandDegrees = -80.0)
            }
            assertEquals("true", tilted["motionValid"])
            assertEquals("false", tilted["valueValid"])
            assertEquals("CAMERA_NOT_DOWNWARD", tilted["reason"])
            assertEquals("null", tilted["forwardMps"])
            assertEquals("null", tilted["rightMps"])

            val nonFinite = observe(diagnostic, frame, results) {
                it.copy(cameraPitchCommandDegrees = Double.NaN)
            }
            assertEquals("true", nonFinite["motionValid"])
            assertEquals("false", nonFinite["valueValid"])
            assertEquals("NON_FINITE_CAMERA_PITCH", nonFinite["reason"])

            val stale = observe(diagnostic, frame, results) {
                it.copy(capturedAtNanos = it.capturedAtNanos - 500_000_000L)
            }
            assertEquals("true", stale["motionValid"])
            assertEquals("false", stale["valueValid"])
            assertEquals("STALE_CONTEXT", stale["reason"])

            // Unsupported camera/context intervals cannot seed propagated metric scale.
            val recovered = observe(diagnostic, frame, results)
            assertEquals("false", recovered["valueValid"])
            assertEquals("BASELINE", recovered["reason"])
            repeat(4) { calibrated = observe(diagnostic, frame, results) }
            assertEquals("true", calibrated?.get("valueValid"))
        } finally {
            diagnostic.close()
            frame.release()
        }
    }

    @Test
    fun enabledSessionReadsCalibratedMotionAndInvalidOutcomesRevokeIt() {
        val results = LinkedBlockingQueue<Map<String, String>>()
        val diagnostic = VisualVelocityDiagnostics { event, _, details ->
            if (event == "visual_velocity_result") {
                results.add(details.split(' ').associate { it.substringBefore('=') to it.substringAfter('=') })
            }
        }
        val frame = calibratedFloor()
        val translated = Mat()
        val transform = Mat(2, 3, CvType.CV_64F)
        val empty = Mat()
        try {
            diagnostic.start("b3", "TRACKING", 0.0, controlFeedback = true)
            assertNull(diagnostic.controlSample("b3"))
            repeat(5) { observe(diagnostic, frame, results) }
            val baseline = checkNotNull(diagnostic.controlSample("b3"))
            assertEquals(0.0, checkNotNull(baseline.forwardMps), 0.005)
            assertEquals(0.0, checkNotNull(baseline.rightMps), 0.005)
            transform.put(0, 0, 1.0, 0.0, 4.0, 0.0, 1.0, 6.0)
            Imgproc.warpAffine(frame, translated, transform, frame.size())
            Thread.sleep(100)
            // Fixed capture interval keeps the consumer speed independent of device processing time.
            diagnostic.submitRgba(translated, baseline.frameNanos + 100_000_000L, context())
            val motion = checkNotNull(results.poll(10, TimeUnit.SECONDS))
            val sample = checkNotNull(diagnostic.controlSample("b3"))
            val intervalSeconds = (sample.frameNanos - baseline.frameNanos) * 1e-9
            // The 100px marker spacing is 0.20m: image motion (+4,+6)px
            // corresponds to body displacement (+0.012,-0.008)m.
            assertEquals(0.012, checkNotNull(sample.forwardMps) * intervalSeconds, 0.001)
            assertEquals(-0.008, checkNotNull(sample.rightMps) * intervalSeconds, 0.001)
            assertEquals(0.0, checkNotNull(sample.aircraftHeadingDegrees), 0.0)
            assertNull(sample.reason)
            assertEquals("true", motion["controlFeedback"])
            assertEquals("true", motion["controlSampleOffered"])
            assertNull(diagnostic.controlSample("b2"))

            val controllers = listOf(
                FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16_VISUAL to 1.90,
                FixedHeadingActuationPhaseLead.SPEED_SCHEDULED_VISUAL to 2.35,
            ).map { (profile, expectedTarget) ->
                TapeTrackingController().apply {
                    start(
                        baseline.frameNanos,
                        TapeTrackingMode.FIXED_HEADING,
                        fixedHeadingActuationPhaseLead = profile,
                        fixedHeadingSpeedTarget = FixedHeadingSpeedTarget.STEP_085,
                    )
                } to expectedTarget
            }
            val path = TapeCenterlinePath(
                sourceWidth = 640,
                sourceHeight = 360,
                xFractions = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
                yFractions = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
                anchorXFraction = 0.5f,
                anchorYFraction = 0.94f,
                lookaheadXFraction = null,
                lookaheadYFraction = null,
                quality = PathQuality.FULL_PATH,
                rejection = null,
            )
            val observation = TapeTrackingObservation(
                angleFromVerticalDegrees = 0.0,
                longSideFraction = 0.8,
                nearFieldOffsetFraction = 0.0,
                bounds = NormalizedRect(0.48, 0.10, 0.52, 0.90),
                lookahead = TapeLookahead(0.5, 0.60),
                quality = PathQuality.FULL_PATH,
                endpointCandidate = false,
                closedLoop = false,
                frameWidthPixels = 640,
                frameHeightPixels = 360,
                heightAboveGroundMeters = 1.2,
                confidence = 0.9,
                centerline = path,
                // Contradictory SDK feedback must not replace the visual measurement.
                actualTravelDirectionDegrees = 0.0,
                actualGroundSpeedMetersPerSecond = 0.70,
                speedFeedbackSampleAtNanos = sample.frameNanos,
            )
            fun consume(
                controller: TapeTrackingController,
                input: VisualVelocityDiagnostics.ControlSample,
            ): TapeTrackingDecision {
                val now = input.frameNanos + 1L
                controller.updateVisualVelocity(
                    input.forwardMps, input.rightMps, input.aircraftHeadingDegrees,
                    0.0, input.frameNanos, now, input.reason,
                )
                controller.observe(observation, now)
                return controller.tick(now)
            }
            controllers.forEach { (controller, expectedTarget) ->
                controller.observe(observation, sample.frameNanos)
                val unmeasured = controller.tick(sample.frameNanos)
                assertEquals(1.60, unmeasured.commandTargetSpeedMetersPerSecond, 1e-9)
                val driven = consume(controller, sample)
                assertEquals(0.12, checkNotNull(driven.measuredAlongTrackSpeedMetersPerSecond), 0.01)
                assertEquals(expectedTarget, driven.commandTargetSpeedMetersPerSecond, 1e-9)
                assertTrue(driven.commandTargetSpeedMetersPerSecond > unmeasured.commandTargetSpeedMetersPerSecond)
            }

            // Controller class loading must not turn a context-rejection test into
            // a frame-gap test; keep the image timestamps independent of wall time.
            val invalid = observe(
                diagnostic, translated, results,
                frameNanos = sample.frameNanos + 100_000_000L,
            ) {
                it.copy(cameraPitchCommandPending = true)
            }
            val revoked = checkNotNull(diagnostic.controlSample("b3"))
            assertTrue(revoked.frameNanos > sample.frameNanos)
            assertNull(revoked.forwardMps)
            assertNull(revoked.rightMps)
            assertEquals("CAMERA_PITCH_PENDING", revoked.reason)
            assertEquals("false", invalid["controlSampleOffered"])
            controllers.forEach { (controller, _) ->
                val withoutFeedback = consume(controller, revoked)
                assertNull(withoutFeedback.measuredAlongTrackSpeedMetersPerSecond)
                assertEquals(0.0, withoutFeedback.speedFeedbackBoostMetersPerSecond, 0.0)
                assertEquals(1.60, withoutFeedback.commandTargetSpeedMetersPerSecond, 1e-9)
            }

            repeat(5) { observe(diagnostic, translated, results) }
            assertEquals(0.0, checkNotNull(diagnostic.controlSample("b3")?.forwardMps), 0.005)
            observe(diagnostic, empty, results)
            val failed = checkNotNull(diagnostic.controlSample("b3"))
            assertNull(failed.forwardMps)
            assertNull(failed.rightMps)
            assertEquals("SNAPSHOT_ERROR", failed.reason)

            // Same attempt ID must not retain an enabled generation's sample.
            diagnostic.start("b3", "TRACKING", 0.0)
            assertNull(diagnostic.controlSample("b3"))
            var diagnosticObservation: Map<String, String>? = null
            repeat(5) { diagnosticObservation = observe(diagnostic, frame, results) }
            assertEquals("true", diagnosticObservation?.get("valueValid"))
            assertEquals("false", diagnosticObservation?.get("controlFeedback"))
            assertEquals("false", diagnosticObservation?.get("controlSampleOffered"))
            assertNull(diagnostic.controlSample("b3"))
        } finally {
            diagnostic.close()
            frame.release()
            translated.release()
            transform.release()
            empty.release()
        }
    }

    @Test
    fun racingHeadingValidityGatesConsumptionWithoutDiscardingRegisteredMarkerScale() {
        val results = LinkedBlockingQueue<Map<String, String>>()
        val diagnostic = VisualVelocityDiagnostics { event, _, details ->
            if (event == "visual_velocity_result") {
                results.add(details.split(' ').associate { it.substringBefore('=') to it.substringAfter('=') })
            }
        }
        val frame = calibratedFloor()
        fun submit(
            headingAgeAtFrame: Long = 0L,
            frameAgeAtSubmission: Long = 0L,
            cameraPitchPending: Boolean = false,
        ): Map<String, String> {
            // Leave room for delayed frames to remain monotonic and admitted.
            Thread.sleep(100 + TimeUnit.NANOSECONDS.toMillis(frameAgeAtSubmission))
            val now = System.nanoTime()
            val frameNanos = now - frameAgeAtSubmission
            diagnostic.submitRgba(
                frame, frameNanos,
                context().copy(
                    aircraftHeadingDegrees = 36.0,
                    aircraftHeadingReceivedAtNanos = frameNanos - headingAgeAtFrame,
                    cameraPitchCommandPending = cameraPitchPending,
                ),
            )
            return checkNotNull(results.poll(10, TimeUnit.SECONDS)) { "racing result did not arrive" }
        }
        fun assertRecovered() {
            submit()
            val sample = checkNotNull(diagnostic.controlSample("racing"))
            assertNull(sample.reason)
            assertEquals(0.0, checkNotNull(sample.forwardMps), 0.005)
            assertEquals(36.0, checkNotNull(sample.aircraftHeadingDegrees), 0.0)
        }
        fun assertRevoked(result: Map<String, String>, reason: String) {
            assertEquals("true", result["motionValid"])
            assertEquals("false", result["valueValid"])
            val sample = checkNotNull(diagnostic.controlSample("racing"))
            assertEquals(reason, sample.reason)
            assertNull(sample.forwardMps)
            assertNull(sample.rightMps)
            assertRecovered()
        }
        try {
            diagnostic.start(
                "racing", "TRACKING", null, controlFeedback = true,
                maximumHeadingAgeNanos = TapeTrackingController.CIRCULAR_VISUAL_MAX_HEADING_AGE_NANOS,
            )
            repeat(5) { submit() }
            assertRecovered()
            // The recorded telemetry cadence has a 216ms median; a normal cached
            // heading must remain usable without extrapolating it from yaw commands.
            submit(headingAgeAtFrame = 216_000_000L)
            assertNull(checkNotNull(diagnostic.controlSample("racing")).reason)
            assertRevoked(submit(headingAgeAtFrame = 251_000_000L), "STALE_AIRCRAFT_HEADING")
            // Fresh at submission, but acquired AFTER this image's admission.
            assertRevoked(
                submit(headingAgeAtFrame = -10_000_000L, frameAgeAtSubmission = 20_000_000L),
                "FUTURE_AIRCRAFT_HEADING",
            )
            // Fresh relative to the image, but not when that image is submitted.
            assertRevoked(
                submit(headingAgeAtFrame = 200_000_000L, frameAgeAtSubmission = 100_000_000L),
                "STALE_AIRCRAFT_HEADING",
            )
            assertRevoked(submit(headingAgeAtFrame = -100_000_000L), "FUTURE_AIRCRAFT_HEADING")

            // Unlike heading age, a camera-change interval invalidates the ground
            // projection: the next supported image cannot reuse the old scale.
            submit(cameraPitchPending = true)
            assertNull(checkNotNull(diagnostic.controlSample("racing")).forwardMps)
            submit()
            assertNull(checkNotNull(diagnostic.controlSample("racing")).forwardMps)
            repeat(4) { submit() }
            assertRecovered()

            // Replacing the strict scope must restore the unchanged legacy timing,
            // not leave a global 250ms gate or allow the old owner to stop the new one.
            diagnostic.start("legacy", "TRACKING", 0.0, controlFeedback = true)
            diagnostic.stop("racing")
            assertNull(diagnostic.controlSample("racing"))
            repeat(5) { submit(headingAgeAtFrame = 300_000_000L) }
            val legacy = checkNotNull(diagnostic.controlSample("legacy"))
            assertNull(legacy.reason)
            assertEquals(0.0, checkNotNull(legacy.forwardMps), 0.005)
            submit(headingAgeAtFrame = -10_000_000L, frameAgeAtSubmission = 20_000_000L)
            assertNull(checkNotNull(diagnostic.controlSample("legacy")).reason)
        } finally {
            diagnostic.close()
            frame.release()
        }
    }

    @Test
    fun replacementDuringResultLoggingCannotResurrectOldControlSample() {
        val blockNextResult = AtomicBoolean(false)
        val resultEntered = CountDownLatch(1)
        val releaseResult = CountDownLatch(1)
        val results = LinkedBlockingQueue<Map<String, String>>()
        val diagnostic = VisualVelocityDiagnostics { event, _, details ->
            if (event == "visual_velocity_result") {
                val fields = details.split(' ').associate { it.substringBefore('=') to it.substringAfter('=') }
                if (fields["attemptId"] == "old" && blockNextResult.compareAndSet(true, false)) {
                    resultEntered.countDown()
                    check(releaseResult.await(10, TimeUnit.SECONDS))
                }
                results.add(fields)
            }
        }
        val frame = calibratedFloor()
        try {
            diagnostic.start("old", "TRACKING", 0.0, controlFeedback = true)
            repeat(5) { observe(diagnostic, frame, results) }
            blockNextResult.set(true)
            Thread.sleep(100)
            val blockedFrameNanos = System.nanoTime()
            diagnostic.submitRgba(frame, blockedFrameNanos, context())
            assertTrue("result callback did not arrive", resultEntered.await(10, TimeUnit.SECONDS))
            // Publication must precede the callback and must not hold the lifecycle lock.
            val offered = checkNotNull(diagnostic.controlSample("old"))
            assertEquals(blockedFrameNanos, offered.frameNanos)
            assertEquals(0.0, checkNotNull(offered.forwardMps), 0.005)

            diagnostic.start("new", "ACQUIRING", 0.0, controlFeedback = true)
            diagnostic.stop("old")
            diagnostic.updatePhase("DISABLED", "old")
            assertNull(diagnostic.controlSample("old"))
            assertNull(diagnostic.controlSample("new"))
            releaseResult.countDown()
            assertEquals("old", results.poll(10, TimeUnit.SECONDS)?.get("attemptId"))
            val replacement = observe(diagnostic, frame, results)
            assertEquals("new", replacement["attemptId"])
            assertEquals("ACQUIRING", replacement["phase"])
            val fresh = checkNotNull(diagnostic.controlSample("new"))
            assertTrue(fresh.frameNanos > blockedFrameNanos)
            assertEquals("BASELINE", fresh.reason)
            assertNull(fresh.forwardMps)
            assertNull(fresh.rightMps)
            assertNull(diagnostic.controlSample("old"))

            repeat(4) { observe(diagnostic, frame, results) }
            val current = checkNotNull(diagnostic.controlSample("new"))
            assertEquals(0.0, checkNotNull(current.forwardMps), 0.005)
            diagnostic.stop("old")
            assertEquals(current, diagnostic.controlSample("new"))
            diagnostic.stop("new")
            assertNull(diagnostic.controlSample("new"))
            diagnostic.start("new", "TRACKING", 0.0, controlFeedback = true)
            assertNull(diagnostic.controlSample("new"))
            repeat(5) { observe(diagnostic, frame, results) }
            assertEquals(0.0, checkNotNull(diagnostic.controlSample("new")?.forwardMps), 0.005)
            diagnostic.close()
            assertNull(diagnostic.controlSample("new"))
        } finally {
            releaseResult.countDown()
            diagnostic.close()
            frame.release()
        }
    }

    private fun observe(
        diagnostic: VisualVelocityDiagnostics,
        frame: Mat,
        results: LinkedBlockingQueue<Map<String, String>>,
        frameNanos: Long? = null,
        changeContext: (VisualVelocityDiagnostics.FrameContext) -> VisualVelocityDiagnostics.FrameContext = { it },
    ): Map<String, String> {
        Thread.sleep(100)
        val currentContext = changeContext(context())
        diagnostic.submitRgba(frame, frameNanos ?: System.nanoTime(), currentContext)
        return checkNotNull(results.poll(10, TimeUnit.SECONDS)) { "visual velocity result did not arrive" }
    }

    private fun calibratedFloor(): Mat {
        val width = 848
        val height = 480
        val random = Random(20260910L)
        val pixels = ByteArray(width * height * 4)
        for (top in 0 until height step 8) {
            for (left in 0 until width step 8) {
                val brightness = 90 + random.nextInt(136)
                for (y in top until minOf(top + 8, height)) {
                    for (x in left until minOf(left + 8, width)) {
                        val index = (y * width + x) * 4
                        pixels[index] = brightness.toByte()
                        pixels[index + 1] = (brightness * 0.68).toInt().toByte()
                        pixels[index + 2] = (brightness * 0.28).toInt().toByte()
                        pixels[index + 3] = 255.toByte()
                    }
                }
            }
        }
        return Mat(height, width, CvType.CV_8UC4).also {
            it.put(0, 0, pixels)
            Imgproc.GaussianBlur(it, it, Size(3.0, 3.0), 0.0)
            val yellow = Scalar(245.0, 225.0, 30.0, 255.0)
            Imgproc.circle(it, Point(374.0, 240.0), 5, yellow, -1)
            Imgproc.circle(it, Point(474.0, 240.0), 5, yellow, -1)
        }
    }

    private fun context(): VisualVelocityDiagnostics.FrameContext {
        val now = System.nanoTime()
        return VisualVelocityDiagnostics.FrameContext(
            null, 0L, "UNAVAILABLE", -90.0, 0.0, null, now,
            aircraftHeadingReceivedAtNanos = now,
        )
    }
}
