package com.durendal.droneagent.lite

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Single-owner, bounded-work geometry using the exact frame's valid VO scale.
 * This inherits VO's fixed-nadir similarity model, not a calibrated 3D camera
 * projection. No height/FOV, gimbal or aircraft telemetry supplies missing scale.
 * Noise floors remain provisional, not calibration.
 */
internal class FixedRadiusCircleFitter(private val config: CircleModelConfig = CircleModelConfig()) {
    private val forward = DoubleArray(MAX_FIT_POINTS)
    private val right = DoubleArray(MAX_FIT_POINTS)
    private val inliers = BooleanArray(MAX_FIT_POINTS)
    private var centerForward = 0.0
    private var centerRight = 0.0
    private var bestCost = Double.POSITIVE_INFINITY

    fun fit(
        path: TapeCenterlinePath,
        scale: CircleImageScale,
        sampleAtNanos: Long,
        nowNanos: Long,
    ): CircleFitResult {
        if (sampleAtNanos <= 0L || nowNanos < sampleAtNanos) return rejected("invalid_or_future_image")
        if (nowNanos - sampleAtNanos > config.maximumVisionAgeNanos) return rejected("stale_image")
        if (scale.sampleAtNanos <= 0L || scale.sampleAtNanos > nowNanos) {
            return rejected("invalid_or_future_image_scale")
        }
        if (scale.sampleAtNanos != sampleAtNanos) return rejected("mismatched_image_scale")
        if (scale.headingAtNanos <= 0L || scale.headingAtNanos > sampleAtNanos) {
            return rejected("missing_or_future_image_heading")
        }
        if (sampleAtNanos - scale.headingAtNanos > config.maximumHeadingAgeNanos) {
            return rejected("stale_image_heading")
        }
        if (!scale.metersPerPixel.isFinite() || scale.metersPerPixel <= 0.0 ||
            !scale.headingDegrees.isFinite() || scale.analysisWidth <= 0 || scale.analysisHeight <= 0
        ) return rejected("invalid_image_scale")
        // The path and VO may use different resolutions, but must describe the
        // same uncropped image. Each resized dimension may round by half a pixel;
        // the relative bound also rules out gross distortion at tiny resolutions.
        val widthResize = scale.analysisWidth.toDouble() / path.sourceWidth
        val heightResize = scale.analysisHeight.toDouble() / path.sourceHeight
        val resizeDifference = abs(widthResize - heightResize)
        val roundingTolerance = 0.5 / path.sourceWidth + 0.5 / path.sourceHeight
        if (resizeDifference > roundingTolerance + 1e-12 ||
            resizeDifference > min(widthResize, heightResize) * MAX_ASPECT_DISTORTION_FRACTION
        ) return rejected("mismatched_image_aspect")
        // Raw skeleton branches include medial spurs. The existing quality policy
        // already checks route coverage before declaring a branch ambiguous.
        untrustedPathReason(path)?.let { return rejected(it) }
        // Inspect the whole bounded input, not just the subsequently sampled points.
        for (index in 0 until path.pointCount) {
            if (!path.xFractions[index].isFinite() || !path.yFractions[index].isFinite() ||
                path.xFractions[index] !in 0f..1f || path.yFractions[index] !in 0f..1f
            ) return rejected("invalid_image_coordinates")
        }

        val metersAcross = scale.analysisWidth * scale.metersPerPixel
        val metersDown = scale.analysisHeight * scale.metersPerPixel
        if (!metersAcross.isFinite() || !metersDown.isFinite()) return rejected("invalid_image_scale")
        var count = 0
        val sampledCount = min(path.pointCount, MAX_FIT_POINTS)
        for (sample in 0 until sampledCount) {
            val index = sample * (path.pointCount - 1) / (sampledCount - 1)
            val f = (0.5 - path.yFractions[index]) * metersDown
            val r = (path.xFractions[index] - 0.5) * metersAcross
            // Dense skeleton pixels are not independent measurements. Deduplicate
            // against all retained samples, including non-adjacent repeated points.
            var independent = true
            for (previous in 0 until count) {
                if (hypot(f - forward[previous], r - right[previous]) < MIN_SUPPORT_SEPARATION_METERS) {
                    independent = false
                    break
                }
            }
            if (independent) {
                forward[count] = f
                right[count] = r
                count++
            }
        }
        if (count < MIN_INLIERS) return rejected("insufficient_independent_support")
        bestCost = Double.POSITIVE_INFINITY
        if (!searchCandidates(count, checkAmbiguity = false)) return rejected("no_fixed_radius_candidate")
        if (!refineCenter(count)) return rejected("ill_conditioned_circle")
        bestCost = cost(centerForward, centerRight, count)
        if (searchCandidates(count, checkAmbiguity = true)) return rejected("ambiguous_circle_center")

        var inlierCount = 0
        var squaredError = 0.0
        var ff = 0.0
        var fr = 0.0
        var rr = 0.0
        var scaleGradientForward = 0.0
        var scaleGradientRight = 0.0
        var previousAngle = 0.0
        var signedSpan = 0.0
        var absoluteSpan = 0.0
        var maximumGap = 0.0
        for (index in 0 until count) {
            val df = centerForward - forward[index]
            val dr = centerRight - right[index]
            val distance = hypot(df, dr)
            val residual = distance - config.radiusMeters
            inliers[index] = abs(residual) <= INLIER_METERS
            if (!inliers[index]) continue
            val angle = atan2(dr, df)
            if (inlierCount > 0) {
                var delta = angle - previousAngle
                if (delta > PI) delta -= 2.0 * PI
                if (delta < -PI) delta += 2.0 * PI
                signedSpan += delta
                absoluteSpan += abs(delta)
                maximumGap = max(maximumGap, abs(delta))
            }
            previousAngle = angle
            inlierCount++
            squaredError += residual * residual
            val uf = df / distance
            val ur = dr / distance
            ff += uf * uf
            fr += uf * ur
            rr += ur * ur
            val projectedPosition = uf * forward[index] + ur * right[index]
            scaleGradientForward += uf * projectedPosition
            scaleGradientRight += ur * projectedPosition
        }
        if (inlierCount < MIN_INLIERS || inlierCount < count * MIN_INLIER_FRACTION) {
            return rejected("too_many_circle_outliers")
        }
        val rms = sqrt(squaredError / inlierCount)
        if (!rms.isFinite() || rms > MAX_RESIDUAL_RMS_METERS) return rejected("non_circular_residual")
        val span = abs(signedSpan)
        if (span < MIN_ARC_SPAN_RADIANS) return rejected("insufficient_arc_coverage")
        if (maximumGap > MAX_ARC_GAP_RADIANS || span > 2.0 * PI + 0.1 ||
            span < absoluteSpan * MIN_TURN_CONSISTENCY
        ) return rejected("discontinuous_or_reversing_arc")
        val determinant = ff * rr - fr * fr
        val smallestInformation = (ff + rr - hypot(ff - rr, 2.0 * fr)) / 2.0
        if (smallestInformation < MIN_INFORMATION_PER_POINT * inlierCount || determinant <= 0.0) {
            return rejected("ill_conditioned_circle")
        }
        // A known radius is a hypothesis, not permission to bend a straight line
        // or wrong-radius arc onto it. An independently free-radius local fit must
        // agree; near-straight support makes that fit singular or drives it away.
        if (!hasConsistentFreeRadius(count, inlierCount)) return rejected("inconsistent_circle_radius")

        // Inverse radial-residual information, with an independent pixel/tape
        // noise floor and correlated translation/scale/yaw uncertainty that does
        // NOT vanish by submitting more skeleton points.
        val pixelNoise = 2.0 * scale.metersPerPixel
        val independentStd = max(MIN_RADIAL_NOISE_METERS, pixelNoise)
        val variance = max(squaredError / (inlierCount - 2), independentStd * independentStd)
        val commonStd = 0.015 + pixelNoise
        val commonVariance = commonStd * commonStd
        val scaleVariance = NOMINAL_SCALE_STD_FRACTION * NOMINAL_SCALE_STD_FRACTION
        val yawVariance = NOMINAL_YAW_STD_RADIANS * NOMINAL_YAW_STD_RADIANS
        // One VO scale perturbs ALL points together. Differentiate the fixed-
        // radius normal equations with respect to that shared fractional scale;
        // the sensitivity is not simply the centre (the radius stays fixed).
        // Add this rank-one covariance once, never divided by the inlier count.
        val scaleSensitivityForward = (rr * scaleGradientForward - fr * scaleGradientRight) / determinant
        val scaleSensitivityRight = (ff * scaleGradientRight - fr * scaleGradientForward) / determinant
        val forwardVariance = variance * rr / determinant + commonVariance +
            scaleVariance * scaleSensitivityForward * scaleSensitivityForward + yawVariance * centerRight * centerRight
        val rightVariance = variance * ff / determinant + commonVariance +
            scaleVariance * scaleSensitivityRight * scaleSensitivityRight + yawVariance * centerForward * centerForward
        val covariance = -variance * fr / determinant +
            scaleVariance * scaleSensitivityForward * scaleSensitivityRight - yawVariance * centerForward * centerRight
        val largestVariance = (forwardVariance + rightVariance +
            hypot(forwardVariance - rightVariance, 2.0 * covariance)) / 2.0
        if (!largestVariance.isFinite() || sqrt(largestVariance) > config.maximumPositionStdMeters) {
            return rejected("uncertain_circle_center")
        }
        return CircleFitResult(
            observation = CircleArcObservation(
                centerForwardMeters = centerForward,
                centerRightMeters = centerRight,
                centerForwardVariance = forwardVariance,
                centerRightVariance = rightVariance,
                centerCovariance = covariance,
                residualRmsMeters = rms,
                inlierCount = inlierCount,
                pointCount = count,
                arcSpanRadians = span,
                sampleAtNanos = sampleAtNanos,
                headingDegrees = scale.headingDegrees,
                headingAtNanos = scale.headingAtNanos,
            ),
        )
    }

    /** Deterministic two-point hypotheses; <= 32 choose 2 times two centres. */
    private fun searchCandidates(count: Int, checkAmbiguity: Boolean): Boolean {
        val seeds = min(count, MAX_SEED_POINTS)
        var found = false
        val radius = config.radiusMeters
        for (a in 0 until seeds - 1) {
            val i = a * (count - 1) / (seeds - 1)
            for (b in a + 1 until seeds) {
                val j = b * (count - 1) / (seeds - 1)
                val df = forward[j] - forward[i]
                val dr = right[j] - right[i]
                val chord = hypot(df, dr)
                if (chord < radius * MIN_SEED_CHORD_RADIUS_RATIO || chord >= 2.0 * radius) continue
                val offset = sqrt(radius * radius - chord * chord / 4.0)
                val midpointF = (forward[i] + forward[j]) / 2.0
                val midpointR = (right[i] + right[j]) / 2.0
                for (side in -1..1 step 2) {
                    val f = midpointF - side * dr / chord * offset
                    val r = midpointR + side * df / chord * offset
                    val candidateCost = cost(f, r, count)
                    if (checkAmbiguity) {
                        if (hypot(f - centerForward, r - centerRight) > radius * 0.35 &&
                            candidateCost <= bestCost + count * MIN_RADIAL_NOISE_METERS * MIN_RADIAL_NOISE_METERS
                        ) return true
                    } else if (candidateCost < bestCost) {
                        bestCost = candidateCost
                        centerForward = f
                        centerRight = r
                        found = true
                    }
                }
            }
        }
        return found
    }

    private fun cost(f: Double, r: Double, count: Int): Double {
        var sum = 0.0
        for (index in 0 until count) {
            val residual = hypot(f - forward[index], r - right[index]) - config.radiusMeters
            sum += min(residual * residual, INLIER_METERS * INLIER_METERS)
        }
        return sum
    }

    private fun refineCenter(count: Int): Boolean {
        repeat(REFINEMENT_STEPS) {
            var ff = 0.0
            var fr = 0.0
            var rr = 0.0
            var gradientF = 0.0
            var gradientR = 0.0
            var support = 0
            for (index in 0 until count) {
                val df = centerForward - forward[index]
                val dr = centerRight - right[index]
                val distance = hypot(df, dr)
                val residual = distance - config.radiusMeters
                if (abs(residual) > INLIER_METERS || distance < 1e-9) continue
                val uf = df / distance
                val ur = dr / distance
                ff += uf * uf
                fr += uf * ur
                rr += ur * ur
                gradientF += uf * residual
                gradientR += ur * residual
                support++
            }
            val determinant = ff * rr - fr * fr
            if (support < MIN_INLIERS || determinant < 1e-8) return false
            val stepF = (rr * gradientF - fr * gradientR) / determinant
            val stepR = (ff * gradientR - fr * gradientF) / determinant
            if (!stepF.isFinite() || !stepR.isFinite() || hypot(stepF, stepR) > config.radiusMeters / 2.0) return false
            // Reject a worsening full step instead of letting an outlier move
            // the solution away from the robust hypothesis.
            val nextF = centerForward - stepF
            val nextR = centerRight - stepR
            if (cost(nextF, nextR, count) > cost(centerForward, centerRight, count) + 1e-12) return true
            centerForward = nextF
            centerRight = nextR
            if (hypot(stepF, stepR) < 1e-7) return true
        }
        return true
    }

    private fun hasConsistentFreeRadius(count: Int, inlierCount: Int): Boolean {
        var f = centerForward
        var r = centerRight
        var radius = config.radiusMeters
        repeat(REFINEMENT_STEPS) {
            var sumF = 0.0
            var sumR = 0.0
            var sumResidual = 0.0
            var ff = 0.0
            var fr = 0.0
            var rr = 0.0
            var gradientF = 0.0
            var gradientR = 0.0
            for (index in 0 until count) {
                if (!inliers[index]) continue
                val df = f - forward[index]
                val dr = r - right[index]
                val distance = hypot(df, dr)
                if (distance < 1e-9) return false
                val uf = df / distance
                val ur = dr / distance
                val residual = distance - radius
                sumF += uf
                sumR += ur
                sumResidual += residual
                ff += uf * uf
                fr += uf * ur
                rr += ur * ur
                gradientF += uf * residual
                gradientR += ur * residual
            }
            ff -= sumF * sumF / inlierCount
            fr -= sumF * sumR / inlierCount
            rr -= sumR * sumR / inlierCount
            gradientF -= sumF * sumResidual / inlierCount
            gradientR -= sumR * sumResidual / inlierCount
            val determinant = ff * rr - fr * fr
            if (determinant <= 1e-8 * inlierCount * inlierCount) return false
            val stepF = -(rr * gradientF - fr * gradientR) / determinant
            val stepR = -(ff * gradientR - fr * gradientF) / determinant
            val stepRadius = (sumResidual + sumF * stepF + sumR * stepR) / inlierCount
            if (!stepRadius.isFinite() || !stepF.isFinite() || !stepR.isFinite()) return false
            f += stepF
            r += stepR
            radius += stepRadius
            if (radius !in config.radiusMeters * 0.5..config.radiusMeters * 1.5 ||
                hypot(f - centerForward, r - centerRight) > config.radiusMeters * 0.5
            ) return false
        }
        return abs(radius - config.radiusMeters) <= config.radiusMeters * MAX_RADIUS_ERROR_FRACTION
    }

    private fun rejected(reason: String) = CircleFitResult(reason = reason)

    internal companion object {
        fun untrustedPathReason(path: TapeCenterlinePath): String? = when {
            path.branchCount < 0 || path.quality == PathQuality.LOST ||
                (path.quality == PathQuality.FULL_PATH && path.rejection != null) ||
                (path.quality == PathQuality.NEAR_FIELD_ONLY && path.rejection != "INSUFFICIENT_LOOKAHEAD") ->
                "ambiguous_or_untrusted_path"
            path.pointCount < MIN_INLIERS || path.pointCount > MAX_INPUT_POINTS ->
                "insufficient_or_excessive_path_support"
            else -> null
        }

        private const val MAX_INPUT_POINTS = 4096
        private const val MAX_FIT_POINTS = 64
        private const val MAX_SEED_POINTS = 32
        private const val REFINEMENT_STEPS = 8
        private const val MIN_INLIERS = 10
        private const val MIN_INLIER_FRACTION = 0.80
        private const val MIN_SUPPORT_SEPARATION_METERS = 0.02
        private const val MIN_SEED_CHORD_RADIUS_RATIO = 0.35
        private const val MIN_ARC_SPAN_RADIANS = 0.85
        private const val MAX_ARC_GAP_RADIANS = 0.30
        private const val MIN_TURN_CONSISTENCY = 0.90
        private const val MIN_INFORMATION_PER_POINT = 0.04
        private const val INLIER_METERS = 0.025
        private const val MAX_RESIDUAL_RMS_METERS = 0.012
        private const val MAX_RADIUS_ERROR_FRACTION = 0.15
        private const val MIN_RADIAL_NOISE_METERS = 0.008
        private const val NOMINAL_SCALE_STD_FRACTION = 0.02
        private const val NOMINAL_YAW_STD_RADIANS = PI / 180.0
        private const val MAX_ASPECT_DISTORTION_FRACTION = 0.01
    }
}
