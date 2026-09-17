package com.durendal.droneagent.lite

/** Main-thread adjunct join; raw detections always reach the racer without waiting for VO. */
internal class CircleVisualFrameJoiner {
    private val observations = arrayOfNulls<TapeTrackingObservation>(16)
    private var next = 0
    private var newestFrameNanos = 0L
    private var consumedFrameNanos = 0L
    private var invalidThroughNanos = 0L

    fun clear(startedAtNanos: Long = 0L) {
        observations.fill(null)
        next = 0
        newestFrameNanos = startedAtNanos
        consumedFrameNanos = startedAtNanos
        invalidThroughNanos = startedAtNanos
    }

    /** A delayed invalid outcome must not discard newer geometry already waiting for VO. */
    fun invalidate(sourceAtNanos: Long) {
        invalidThroughNanos = maxOf(invalidThroughNanos, sourceAtNanos)
        discardThrough(invalidThroughNanos)
    }

    /** False withdraws the adjunct immediately, including decoded frames skipped by VO. */
    fun offer(
        observation: TapeTrackingObservation?,
        sourceAtNanos: Long = observation?.capturedAtNanos ?: newestFrameNanos,
    ): Boolean {
        if (sourceAtNanos <= invalidThroughNanos || sourceAtNanos <= consumedFrameNanos) return true
        if (RacingCircleGuidance.untrustedObservationReason(observation) != null) {
            invalidate(sourceAtNanos)
            return false
        }
        val image = checkNotNull(observation)
        if (image.capturedAtNanos <= newestFrameNanos) return true
        newestFrameNanos = image.capturedAtNanos
        observations[next] = image
        next = (next + 1) % observations.size
        return true
    }

    fun take(sample: VisualVelocityDiagnostics.ControlSample): TapeTrackingObservation? {
        if (sample.frameNanos <= consumedFrameNanos || sample.frameNanos <= invalidThroughNanos ||
            sample.reason != null
        ) return null
        val scale = sample.metersPerPixel?.takeIf { it.isFinite() && it > 0.0 } ?: return null
        val heading = sample.aircraftHeadingDegrees?.takeIf(Double::isFinite) ?: return null
        if (sample.analysisWidth <= 0 || sample.analysisHeight <= 0 || sample.aircraftHeadingAtNanos <= 0L ||
            sample.forwardMps?.isFinite() != true || sample.rightMps?.isFinite() != true
        ) return null
        for (index in observations.indices) {
            val observation = observations[index] ?: continue
            if (observation.capturedAtNanos != sample.frameNanos) continue
            consumedFrameNanos = sample.frameNanos
            discardThrough(consumedFrameNanos)
            return observation.copy(
                circleImageScale = CircleImageScale(
                    metersPerPixel = scale,
                    analysisWidth = sample.analysisWidth,
                    analysisHeight = sample.analysisHeight,
                    sampleAtNanos = sample.frameNanos,
                    headingDegrees = heading,
                    headingAtNanos = sample.aircraftHeadingAtNanos,
                ),
            )
        }
        return null
    }

    private fun discardThrough(sourceAtNanos: Long) {
        for (index in observations.indices) {
            if ((observations[index]?.capturedAtNanos ?: Long.MAX_VALUE) <= sourceAtNanos) {
                observations[index] = null
            }
        }
    }
}
