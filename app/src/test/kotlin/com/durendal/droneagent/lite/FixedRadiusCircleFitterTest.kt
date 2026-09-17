package com.durendal.droneagent.lite

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class FixedRadiusCircleFitterTest {
    private val fitter = FixedRadiusCircleFitter()

    @Test
    fun `left and right translated rotated arcs recover ground centres rather than image anchors`() {
        for (side in listOf(-1.0, 1.0)) {
            val rotation = 0.48 * side
            val center = rotate(Point(0.08, side * 0.67), rotation)
            val points = circleArc(Point(0.08, side * 0.67), side = side).map { rotate(it, rotation) }
            val fit = requireFit(path(points))
            assertEquals(center.forward, fit.centerForwardMeters, 0.001)
            assertEquals(center.right, fit.centerRightMeters, 0.001)
            assertTrue(fit.residualRmsMeters < 0.001)
            assertTrue(fit.arcSpanRadians > 1.2)
            assertCredibleCovariance(fit)
        }
    }

    @Test
    fun `image up is forward and image right is right at the measured metric scale`() {
        // Hand-derived fractions for a .004 m/px, 640x360 analysis image. The
        // path source is 1280x720: source pixels must NOT set the metric scale.
        val points = circleArc(Point(0.0, -RADIUS), side = -1.0)
        val xs = FloatArray(points.size) { (0.5 + points[it].right / 2.56).toFloat() }
        val ys = FloatArray(points.size) { (0.5 - points[it].forward / 1.44).toFloat() }
        val fit = requireFit(imagePath(xs, ys))
        assertEquals(0.0, fit.centerForwardMeters, 0.001)
        assertEquals(-RADIUS, fit.centerRightMeters, 0.001)
    }

    @Test
    fun `normalized source coordinates support coherent analysis and source resolution changes`() {
        val center = Point(0.08, RADIUS)
        val points = circleArc(center)
        for (sourceSize in listOf(640 to 360, 1280 to 720, 2560 to 1440)) {
            val image = path(points, sourceWidth = sourceSize.first, sourceHeight = sourceSize.second)
            val fit = requireFit(image)
            assertEquals(center.forward, fit.centerForwardMeters, 0.001)
            assertEquals(center.right, fit.centerRightMeters, 0.001)
            // Twice the analysis resolution has half the metric pixel scale.
            val doubled = scale().copy(metersPerPixel = 0.002, analysisWidth = 1280, analysisHeight = 720)
            val higherResolutionFit = requireFit(image, doubled)
            assertEquals(center.forward, higherResolutionFit.centerForwardMeters, 0.001)
            assertEquals(center.right, higherResolutionFit.centerRightMeters, 0.001)
        }
    }

    @Test
    fun `proportional resize rounding is allowed but anisotropic and tiny distorted images are not`() {
        val points = circleArc(Point(0.0, RADIUS))
        val roundedScale = scale().copy(analysisHeight = 361)
        val rounded = requireFit(path(points, roundedScale, sourceHeight = 721), roundedScale)
        assertEquals(RADIUS, rounded.centerRightMeters, 0.001)
        val image = path(points)
        assertRejected(image, scale().copy(analysisHeight = 480))
        assertRejected(image, scale().copy(analysisHeight = 364))
        assertRejected(image, scale().copy(analysisWidth = 1, analysisHeight = 1))
    }

    @Test
    fun `isolated gross outliers do not drag a noisy supported circle`() {
        val center = Point(0.02, RADIUS)
        val points = circleArc(center, count = 61).mapIndexed { index, point ->
            val angle = -0.7 + 1.4 * index / 60.0
            val noise = if (index in listOf(9, 24, 42, 54)) 0.10 else 0.004 * sin(index * 1.71)
            Point(point.forward + noise * sin(angle), point.right - noise * cos(angle))
        }
        val fit = requireFit(path(points))
        assertEquals(center.forward, fit.centerForwardMeters, 0.015)
        assertEquals(center.right, fit.centerRightMeters, 0.015)
        assertTrue(fit.residualRmsMeters < 0.01)
        assertCredibleCovariance(fit)
        val errorF = fit.centerForwardMeters - center.forward
        val errorR = fit.centerRightMeters - center.right
        val determinant = fit.centerForwardVariance * fit.centerRightVariance - fit.centerCovariance * fit.centerCovariance
        val mahalanobis = (fit.centerRightVariance * errorF * errorF + fit.centerForwardVariance * errorR * errorR -
            2.0 * fit.centerCovariance * errorF * errorR) / determinant
        assertTrue("reported uncertainty must cover ground-truth error", mahalanobis < 9.21)
    }

    @Test
    fun `short visible lookahead is allowed only when independent circle support remains trustworthy`() {
        val points = circleArc(Point(0.0, RADIUS))
        val nearOnly = path(points, quality = PathQuality.NEAR_FIELD_ONLY, rejection = "INSUFFICIENT_LOOKAHEAD")
        val fit = requireFit(nearOnly)
        assertEquals(RADIUS, fit.centerRightMeters, 0.001)
        assertRejected(path(points, quality = PathQuality.NEAR_FIELD_ONLY, rejection = "AMBIGUOUS_BRANCH", branchCount = 1))
        assertRejected(path(points, quality = PathQuality.NEAR_FIELD_ONLY, rejection = null))
        assertEquals(RADIUS, requireFit(path(points, branchCount = 1)).centerRightMeters, 0.001)
        assertRejected(path(points, quality = PathQuality.LOST, rejection = "NO_NEAR_FIELD_COMPONENT"))
    }

    @Test
    fun `straight wrong radius and noncircular evidence cannot be forced onto configured radius`() {
        assertRejected(path((0 until 41).map { Point(-0.5 + it * 0.025, 0.02) }))
        assertRejected(path(circleArc(Point(0.0, 1.3), radius = 1.3, span = 0.86)))
        assertRejected(path(circleArc(Point(0.0, 0.42), radius = 0.42, span = 2.0)))
        val sBend = (0 until 51).map { index ->
            val f = -0.5 + index * 0.02
            Point(f, 0.13 * sin(f * 7.0))
        }
        assertRejected(path(sBend))
    }

    @Test
    fun `short arc dense duplicate support and disconnected fragments fail closed`() {
        assertRejected(path(circleArc(Point(0.0, RADIUS), span = 0.45)))
        val repeated = List(80) { Point(0.01, 0.02) }
        assertRejected(path(repeated))
        val threeLocations = circleArc(Point(0.0, RADIUS), count = 3)
        assertRejected(path(List(90) { threeLocations[it % 3] }))
        val disconnected = circleArc(Point(0.0, RADIUS), span = 1.4, count = 61).filterIndexed { index, _ ->
            index < 20 || index > 40
        }
        assertRejected(path(disconnected))
    }

    @Test
    fun `arc traversal that reverses cannot masquerade as a continuous model support`() {
        val points = circleArc(Point(0.0, RADIUS), count = 41)
        val reversing = points.filterIndexed { index, _ -> index % 2 == 0 } +
            points.filterIndexed { index, _ -> index % 2 == 1 }.reversed()
        assertRejected(path(reversing))
    }

    @Test
    fun `a substantial alternate path cannot be ignored as a few outliers`() {
        val points = circleArc(Point(0.0, RADIUS), count = 61).mapIndexed { index, point ->
            if (index in 20..40) point.copy(right = point.right + 0.14) else point
        }
        assertRejected(path(points))
    }

    @Test
    fun `image timestamps must be real current and not in the future`() {
        val path = path(circleArc(Point(0.0, RADIUS)))
        assertRejectedResult(fitter.fit(path, scale(), 0L, NOW))
        assertRejectedResult(fitter.fit(path, scale().copy(sampleAtNanos = NOW + 1L), NOW + 1L, NOW))
        val old = NOW - 250_000_001L
        val oldScale = scale().copy(sampleAtNanos = old, headingAtNanos = old)
        assertRejectedResult(fitter.fit(path, oldScale, old, NOW))
    }

    @Test
    fun `scale from any other frame is rejected even when both frames are fresh`() {
        val path = path(circleArc(Point(0.0, RADIUS)))
        val sample = NOW - 50_000_000L
        for (scaleSample in listOf(0L, NOW + 1L, sample - 1L, sample + 1L, NOW - 300_000_001L)) {
            val otherFrame = scale().copy(sampleAtNanos = scaleSample, headingAtNanos = sample - 10_000_000L)
            assertRejectedResult(fitter.fit(path, otherFrame, sample, NOW))
        }
    }

    @Test
    fun `image heading must have a real nonfuture unexpired sample timestamp`() {
        val path = path(circleArc(Point(0.0, RADIUS)))
        val sample = NOW - 50_000_000L
        for (headingSample in listOf(0L, NOW + 1L, sample + 1L, sample - 250_000_001L)) {
            val invalid = scale().copy(sampleAtNanos = sample, headingAtNanos = headingSample)
            assertRejectedResult(fitter.fit(path, invalid, sample, NOW))
        }
    }

    @Test
    fun `nonfinite nonpositive scale and unavailable dimensions or heading fail closed`() {
        val path = path(circleArc(Point(0.0, RADIUS)))
        val invalid = listOf(
            scale().copy(metersPerPixel = Double.NaN),
            scale().copy(metersPerPixel = Double.POSITIVE_INFINITY),
            scale().copy(metersPerPixel = 0.0),
            scale().copy(metersPerPixel = -0.004),
            scale().copy(analysisWidth = 0),
            scale().copy(analysisHeight = -1),
            scale().copy(headingDegrees = Double.NaN),
            scale().copy(headingDegrees = Double.POSITIVE_INFINITY),
        )
        for (imageScale in invalid) assertRejected(path, imageScale)
    }

    @Test
    fun `a corrupt coordinate hidden between downsampled locations is not silently ignored`() {
        val path = path(circleArc(Point(0.0, RADIUS), count = 1000))
        path.xFractions[501] = Float.NaN
        assertRejected(path)
        val outside = path(circleArc(Point(0.0, RADIUS)))
        outside.yFractions[3] = 1.1f
        assertRejected(outside)
    }

    @Test
    fun `covariance rotates with geometry and stays meaningful for arbitrarily dense exact pixels`() {
        val center = Point(0.0, RADIUS)
        val points = circleArc(center)
        val base = requireFit(path(points))
        val rotation = 0.5
        val rotated = requireFit(path(points.map { rotate(it, rotation) }))
        val c = cos(rotation)
        val s = sin(rotation)
        assertEquals(c * c * base.centerForwardVariance + s * s * base.centerRightVariance -
            2.0 * c * s * base.centerCovariance, rotated.centerForwardVariance, 1e-7)
        assertEquals(s * s * base.centerForwardVariance + c * c * base.centerRightVariance +
            2.0 * c * s * base.centerCovariance, rotated.centerRightVariance, 1e-7)
        assertEquals(c * s * (base.centerForwardVariance - base.centerRightVariance) +
            (c * c - s * s) * base.centerCovariance, rotated.centerCovariance, 1e-7)
        val dense = requireFit(path(circleArc(center, count = 2000)))
        assertTrue("exact pixels cannot remove shared metric-scale uncertainty", sqrt(dense.centerForwardVariance) > 0.01)
        assertTrue("duplicating pixels cannot manufacture confidence", largestVariance(dense) > largestVariance(base) * 0.5)
        assertCredibleCovariance(dense)
    }

    @Test
    fun `weaker arc and larger ground scale carry greater rather than zero uncertainty`() {
        val center = Point(0.0, RADIUS)
        val wide = requireFit(path(circleArc(center, span = 1.4)))
        val narrow = requireFit(path(circleArc(center, span = 0.92)))
        assertTrue(narrow.centerForwardVariance > wide.centerForwardVariance)
        val coarseScale = scale().copy(metersPerPixel = 0.008)
        val high = requireFit(path(circleArc(center), coarseScale), coarseScale)
        assertTrue(largestVariance(high) > largestVariance(wide))
        val strict = FixedRadiusCircleFitter(CircleModelConfig(maximumPositionStdMeters = 0.005))
        assertRejectedResult(strict.fit(path(circleArc(center)), scale(), NOW, NOW))
    }

    @Test
    fun `fresh image preserves its own measurement and heading times through metric conversion`() {
        val sample = NOW - 80_000_000L
        val imageScale = scale().copy(sampleAtNanos = sample, headingDegrees = -123.0, headingAtNanos = sample - 5_000_000L)
        val result = fitter.fit(path(circleArc(Point(0.0, RADIUS)), imageScale), imageScale, sample, NOW)
        val fit = result.observation
        assertNotNull(result.reason, fit)
        assertEquals(sample, fit!!.sampleAtNanos)
        assertEquals(imageScale.headingAtNanos, fit.headingAtNanos)
        assertEquals(-123.0, fit.headingDegrees, 0.0)
    }

    @Test
    fun `same frame image scale and image heading are accepted exactly at their freshness limits`() {
        val sample = NOW - 250_000_000L
        val imageScale = scale().copy(sampleAtNanos = sample, headingAtNanos = sample - 250_000_000L)
        val result = fitter.fit(path(circleArc(Point(0.0, RADIUS)), imageScale), imageScale, sample, NOW)
        assertNotNull(result.reason, result.observation)
        assertEquals(sample, result.observation!!.sampleAtNanos)
        assertEquals(imageScale.headingAtNanos, result.observation!!.headingAtNanos)
    }

    private fun requireFit(path: TapeCenterlinePath, imageScale: CircleImageScale = scale()): CircleArcObservation {
        val result = fitter.fit(path, imageScale, NOW, NOW)
        assertNotNull("expected an identifiable circle, rejected: ${result.reason}", result.observation)
        assertNull(result.reason)
        return result.observation!!
    }

    private fun assertRejected(path: TapeCenterlinePath, imageScale: CircleImageScale = scale()) =
        assertRejectedResult(fitter.fit(path, imageScale, NOW, NOW))

    private fun assertRejectedResult(result: CircleFitResult) {
        assertNull("unsafe evidence must not produce a circle", result.observation)
        assertTrue("a consumer needs an explicit failure reason", !result.reason.isNullOrBlank())
    }

    private fun assertCredibleCovariance(fit: CircleArcObservation) {
        assertTrue(fit.centerForwardVariance.isFinite() && fit.centerForwardVariance > 0.0)
        assertTrue(fit.centerRightVariance.isFinite() && fit.centerRightVariance > 0.0)
        assertTrue(fit.centerCovariance.isFinite())
        assertTrue(fit.centerForwardVariance * fit.centerRightVariance > fit.centerCovariance * fit.centerCovariance)
        assertTrue(sqrt(largestVariance(fit)) < CircleModelConfig().maximumPositionStdMeters)
    }

    private fun largestVariance(fit: CircleArcObservation) =
        (fit.centerForwardVariance + fit.centerRightVariance +
            hypot(fit.centerForwardVariance - fit.centerRightVariance, 2.0 * fit.centerCovariance)) / 2.0

    private fun scale() = CircleImageScale(
        metersPerPixel = 0.004,
        analysisWidth = 640,
        analysisHeight = 360,
        sampleAtNanos = NOW,
        headingDegrees = 37.0,
        headingAtNanos = NOW,
    )

    private data class Point(val forward: Double, val right: Double)

    private fun circleArc(
        center: Point,
        side: Double = 1.0,
        radius: Double = RADIUS,
        span: Double = 1.4,
        count: Int = 41,
    ) = List(count) { index ->
        val angle = -span / 2.0 + span * index / (count - 1)
        Point(center.forward + radius * sin(angle), center.right - side * radius * cos(angle))
    }

    private fun rotate(point: Point, angle: Double) = Point(
        cos(angle) * point.forward - sin(angle) * point.right,
        sin(angle) * point.forward + cos(angle) * point.right,
    )

    /** Invert the same-frame VO similarity scale to normalized source coordinates. */
    private fun path(
        points: List<Point>,
        imageScale: CircleImageScale = scale(),
        quality: PathQuality = PathQuality.FULL_PATH,
        rejection: String? = null,
        branchCount: Int = 0,
        sourceWidth: Int = 1280,
        sourceHeight: Int = 720,
    ): TapeCenterlinePath {
        val xs = FloatArray(points.size)
        val ys = FloatArray(points.size)
        points.forEachIndexed { index, point ->
            xs[index] = (0.5 + point.right / (imageScale.analysisWidth * imageScale.metersPerPixel)).toFloat()
            ys[index] = (0.5 - point.forward / (imageScale.analysisHeight * imageScale.metersPerPixel)).toFloat()
            assertTrue("fixture point must lie within the image", xs[index] in 0f..1f && ys[index] in 0f..1f)
        }
        return imagePath(xs, ys, quality, rejection, branchCount, sourceWidth, sourceHeight)
    }

    private fun imagePath(
        xs: FloatArray,
        ys: FloatArray,
        quality: PathQuality = PathQuality.FULL_PATH,
        rejection: String? = null,
        branchCount: Int = 0,
        sourceWidth: Int = 1280,
        sourceHeight: Int = 720,
    ) = TapeCenterlinePath(
        sourceWidth = sourceWidth,
        sourceHeight = sourceHeight,
        xFractions = xs,
        yFractions = ys,
        anchorXFraction = 0.5f,
        anchorYFraction = 0.94f,
        lookaheadXFraction = if (quality == PathQuality.FULL_PATH) xs.last() else null,
        lookaheadYFraction = if (quality == PathQuality.FULL_PATH) ys.last() else null,
        quality = quality,
        rejection = rejection,
        branchCount = branchCount,
    )

    private companion object {
        const val RADIUS = 0.75
        const val NOW = 2_000_000_000L
    }
}
