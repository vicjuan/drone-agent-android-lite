package com.durendal.droneagent.lite

/**
 * What the geometry stage concluded about one candidate: how much of it the
 * aircraft may act on, and why it is not more.
 *
 * [lookahead] is present only for [PathQuality.FULL_PATH]. A downgraded verdict
 * never carries a target, because a target the controller must not use is worse
 * than no target: it invites exactly the "it looked fine on the overlay" class of
 * accident this pipeline exists to remove.
 */
internal class TapePathVerdict(
    val quality: PathQuality,
    val lookahead: TapeLookahead?,
    val rejection: TapeCandidateRejection?,
) {
    init {
        require(quality == PathQuality.FULL_PATH || lookahead == null) {
            "only FULL_PATH may carry a look-ahead point"
        }
        require(quality != PathQuality.FULL_PATH || lookahead != null) {
            "FULL_PATH without a look-ahead point is not a full path"
        }
    }
}

/**
 * Decides path quality from observable geometry, topology and the tracking mode.
 *
 * The rules are deliberately one-directional: every condition can only take
 * quality down, never up. There is no branch that recovers a FULL_PATH from
 * partial evidence, which is what makes the outcome auditable from the inputs
 * rather than from the order the checks happen to run in.
 */
internal object TapePathQualityPolicy {

    fun evaluate(
        estimate: CenterlineEstimate,
        measurement: CenterlinePathMeasurement?,
        mode: TapeDetectionMode,
        frameHeight: Int,
        /**
         * How much of the candidate's tape the reported chain accounts for. A
         * junction leaves a whole arm unexplained, and the skeleton's own branch
         * counter can miss that when the arm attaches where the route already
         * runs. Unexplained tape is ambiguity whatever produced it.
         */
        pathCoverage: Double = 1.0,
    ): TapePathVerdict {
        if (estimate.points.size < CenterlineMeasurement.MIN_POINT_COUNT || measurement == null) {
            return TapePathVerdict(
                PathQuality.LOST,
                null,
                TapeCandidateRejection.NO_CENTERLINE,
            )
        }

        // The near field is what an in-place alignment steers by, so if it is not
        // credible there is nothing safe to do with this candidate at all.
        //
        // Credibility is about shape, not where the chain sits in the frame:
        // camera pitch and flight height move a valid near-field segment without
        // changing its geometry. Whether the chain reaches the near edge is a
        // mission question handled by the endpoint rules.
        val nearFieldCredible =
            measurement.arcLengthFraction >= MIN_NEAR_FIELD_ARC_FRACTION &&
                estimate.components.widthConsistency >= MIN_NEAR_FIELD_WIDTH_CONSISTENCY
        if (!nearFieldCredible) {
            return TapePathVerdict(
                PathQuality.LOST,
                null,
                TapeCandidateRejection.NO_NEAR_FIELD_COMPONENT,
            )
        }

        // Skeletonization can leave a long medial spur on one physical ribbon.
        // A branch is actionable ambiguity only when the selected route also
        // leaves a material share of the candidate unexplained. This keeps a
        // high-coverage single path moving while a real T junction still stops.
        // A closed loop is held to the same standard over the half of itself
        // its route can reach: the unexplained remainder is the same tape
        // rejoining itself, not a competing shape.
        val minimumCoverage = when {
            estimate.topology.branchCount > 0 -> MIN_BRANCHED_PATH_COVERAGE
            estimate.topology.closedLoop -> MIN_CLOSED_LOOP_PATH_COVERAGE
            else -> MIN_PATH_COVERAGE
        }
        if (pathCoverage < minimumCoverage) {
            return TapePathVerdict(
                PathQuality.NEAR_FIELD_ONLY,
                null,
                TapeCandidateRejection.AMBIGUOUS_BRANCH,
            )
        }

        val lookahead = measurement.lookahead
            ?: return TapePathVerdict(
                PathQuality.NEAR_FIELD_ONLY,
                null,
                TapeCandidateRejection.INSUFFICIENT_LOOKAHEAD,
            )

        if (measurement.arcLengthFraction < MIN_FULL_PATH_ARC_FRACTION) {
            return TapePathVerdict(
                PathQuality.NEAR_FIELD_ONLY,
                null,
                TapeCandidateRejection.INSUFFICIENT_LOOKAHEAD,
            )
        }

        return TapePathVerdict(PathQuality.FULL_PATH, lookahead, null)
    }

    /**
     * Whether the far end of this chain may be treated as a candidate physical
     * end of tape.
     *
     * A chain cut off by the frame border says the tape left the field of view,
     * which is the opposite of an ending. A closed loop has no end at all. Only a
     * terminus that stops inside the frame is evidence of one, and in circular
     * mode a loop must never be read as an endpoint no matter how the length
     * behaves.
     */
    fun isEndpointCandidate(
        topology: CenterlineTopology,
        mode: TapeDetectionMode,
    ): Boolean = when {
        topology.closedLoop -> false
        mode == TapeDetectionMode.PATH && topology.distalTerminus == CenterlineTerminus.NONE -> false
        else -> topology.distalTerminus == CenterlineTerminus.INSIDE_FRAME
    }

    /**
     * Whether a candidate turns enough to be curved tape rather than a straight
     * wall or floor edge. This gates acquisition only.
     *
     * Deliberately net turn alone. Turn consistency is measured and reported,
     * but it is not a gate: the recorded cardboard scene turns 71 degrees in one
     * direction and still scores only 0.33, because real tape wanders and its
     * skeleton wanders more. Requiring smoothness here rejects real tape, and it
     * would also reject S-shaped tape, which is tape.
     */
    fun isCredibleArc(measurement: CenterlinePathMeasurement): Boolean =
        measurement.totalPathTurnDegrees + ARC_TURN_TOLERANCE_DEGREES >=
            MIN_ARC_TOTAL_TURN_DEGREES

    /** A closed loop is a legal circular path and is never an endpoint. */
    fun isClosedLoopPath(topology: CenterlineTopology): Boolean = topology.closedLoop

    fun isFrameBorderTruncation(topology: CenterlineTopology): Boolean =
        topology.distalTerminus == CenterlineTerminus.AT_FRAME_BORDER

    /** Below this the chain is too short to provide even a bounded yaw direction. */
    const val MIN_NEAR_FIELD_ARC_FRACTION = 0.07

    const val MIN_NEAR_FIELD_WIDTH_CONSISTENCY = 0.30

    /**
     * Conservative on purpose: a chain shorter than this may have a technically
     * usable look-ahead point and still be a residual fragment, and the cost of
     * being wrong is forward motion along something that is not the tape.
     */
    const val MIN_FULL_PATH_ARC_FRACTION = 0.25

    /**
     * Below this, enough of the candidate lies off the reported chain that the
     * extractor is choosing between shapes rather than describing one.
     */
    const val MIN_PATH_COVERAGE = 0.70

    /**
     * A closed loop's route is a longest path from its lowest skeleton point,
     * so it traverses about half the cycle and its coverage sits near 0.5 by
     * construction; the pixels it leaves out are the same tape rejoining
     * itself (which is also why the extractor reports zero branches for a
     * loop). Applying [MIN_PATH_COVERAGE] to the reachable half keeps the
     * same tolerance for genuinely unexplained tape.
     */
    const val MIN_CLOSED_LOOP_PATH_COVERAGE = MIN_PATH_COVERAGE / 2

    /**
     * A reported skeleton branch may be a medial-axis spur, but only when the
     * selected route still explains nearly all candidate pixels. The synthetic
     * T junction measures 0.76; 0.85 keeps it stopped while admitting the
     * high-coverage single ribbon reproduced by the flight failure.
     */
    const val MIN_BRANCHED_PATH_COVERAGE = 0.85

    /**
     * Historical acquisition boundary retained until raw camera captures provide
     * a measured turn distribution. Integer thinning tolerance is applied above.
     */
    const val MIN_ARC_TOTAL_TURN_DEGREES = 8.0

    /**
     * Skeleton turn is sampled from integer pixels. Keep a sub-degree tolerance
     * around the corpus-calibrated boundary so the same visible arc does not
     * alternate across the gate after harmless thinning or resize rounding.
     */
    private const val ARC_TURN_TOLERANCE_DEGREES = 0.5
}
