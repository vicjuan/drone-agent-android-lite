package com.durendal.droneagent.lite

/**
 * One frame's side-by-side record of the estimator that flies today and the
 * replacement centerline, joined by a single frame identifier.
 *
 * Written as one flat line per frame so it can be grepped straight out of the
 * flight log and lined up with a stored capture by `seq` — which is the whole
 * point: a disagreement is only actionable if the exact frame that produced it
 * can be replayed.
 */
internal fun formatShadowComparison(
    frameSequence: Long,
    frameNanos: Long,
    oldAnchorXFraction: Double,
    oldAnchorYFraction: Double,
    oldLookaheadXFraction: Double,
    oldLookaheadYFraction: Double,
    oldNearFieldAngleDegrees: Double,
    oldArcLengthFraction: Double,
    oldMedianWidthFraction: Double,
    oldCurvatureDegrees: Double,
    oldHorizontalFallback: Boolean,
    oldSampleCount: Int,
    newPointCount: Int,
    newConfidence: Double,
    newSupport: Double,
    newWidthConsistency: Double,
    newContinuity: Double,
    newFitResidual: Double,
    newTerminus: CenterlineTerminus,
    newDistalBorderDistancePixels: Double,
    newBranchCount: Int,
    newClosedLoop: Boolean,
    newMeasurement: CenterlinePathMeasurement?,
    maskNanos: Long,
    extractNanos: Long,
    measureNanos: Long,
): String = buildString {
    append("shadow seq=").append(frameSequence)
    append(" frameNanos=").append(frameNanos)
    append(" old=ok")
    append(" oldAnchor=").append(fraction(oldAnchorXFraction)).append(',').append(fraction(oldAnchorYFraction))
    append(" oldLookahead=").append(fraction(oldLookaheadXFraction)).append(',').append(fraction(oldLookaheadYFraction))
    append(" oldAngle=").append(degrees(oldNearFieldAngleDegrees))
    append(" oldCurve=").append(degrees(oldCurvatureDegrees))
    append(" oldArc=").append(fraction(oldArcLengthFraction))
    append(" oldWidth=").append(fraction(oldMedianWidthFraction))
    append(" oldSamples=").append(oldSampleCount)
    append(" oldAxis=").append(if (oldHorizontalFallback) "HORIZONTAL_FALLBACK" else "CENTERLINE")

    val reason = shadowRejectionReason(newPointCount, newMeasurement)
    append(" new=").append(if (reason == null) "ok" else "rejected")
    append(" newPoints=").append(newPointCount)
    append(" newConfidence=").append(fraction(newConfidence))
    append(" newComponents=support:").append(fraction(newSupport))
        .append(",width:").append(fraction(newWidthConsistency))
        .append(",continuity:").append(fraction(newContinuity))
        .append(",fit:").append(fraction(newFitResidual))
    append(" newTerminus=").append(newTerminus.name)
    append(" newBorderPx=").append(degrees(newDistalBorderDistancePixels))
    append(" newBranches=").append(newBranchCount)
    append(" newLoop=").append(newClosedLoop)
    if (newMeasurement == null) {
        append(" newAnchor=none newLookahead=none newAngle=none newCurve=none newArc=none")
    } else {
        append(" newAnchor=").append(fraction(newMeasurement.anchorXFraction))
            .append(',').append(fraction(newMeasurement.anchorYFraction))
        append(" newLookahead=").append(
            newMeasurement.lookahead?.let {
                "${fraction(it.xFraction)},${fraction(it.yFraction)}"
            } ?: "none",
        )
        append(" newAngle=").append(degrees(newMeasurement.nearFieldAngleFromVerticalDegrees))
        append(" newCurve=").append(degrees(newMeasurement.curvatureDegrees))
        append(" newArc=").append(fraction(newMeasurement.arcLengthFraction))
        append(" newWidth=").append(fraction(newMeasurement.medianWidthFraction))
        append(" newQuality=").append(
            if (newMeasurement.lookahead == null) {
                PathQuality.NEAR_FIELD_ONLY.name
            } else {
                PathQuality.FULL_PATH.name
            },
        )
    }
    append(" newReject=").append(reason ?: "none")

    if (newMeasurement != null) {
        append(" dAnchorX=").append(
            fraction(newMeasurement.anchorXFraction - oldAnchorXFraction),
        )
        append(" dAngle=").append(
            degrees(newMeasurement.nearFieldAngleFromVerticalDegrees - oldNearFieldAngleDegrees),
        )
        newMeasurement.lookahead?.let {
            append(" dLookaheadX=").append(fraction(it.xFraction - oldLookaheadXFraction))
            append(" dLookaheadY=").append(fraction(it.yFraction - oldLookaheadYFraction))
        }
    }

    append(" maskMs=").append(millis(maskNanos))
    append(" extractMs=").append(millis(extractNanos))
    append(" measureMs=").append(millis(measureNanos))
    append(" totalMs=").append(millis(maskNanos + extractNanos + measureNanos))
}

/**
 * Why the replacement produced nothing usable, in the vocabulary the redesign
 * document defines. Null means it produced a full path.
 */
internal fun shadowRejectionReason(
    pointCount: Int,
    measurement: CenterlinePathMeasurement?,
): String? = when {
    pointCount == 0 -> TapeCandidateRejection.NO_CENTERLINE.name
    measurement == null -> TapeCandidateRejection.NO_CENTERLINE.name
    measurement.lookahead == null -> TapeCandidateRejection.INSUFFICIENT_LOOKAHEAD.name
    else -> null
}

private fun fraction(value: Double): String = "%.3f".format(value)

private fun degrees(value: Double): String = "%.1f".format(value)

private fun millis(nanos: Long): String = "%.1f".format(nanos / 1_000_000.0)
