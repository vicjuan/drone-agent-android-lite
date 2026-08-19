package com.durendal.droneagent.lite

import java.io.File
import java.util.ArrayDeque
import java.util.concurrent.Executor
import java.util.concurrent.Executors

/**
 * Collects replayable detector evidence around an interesting moment.
 *
 * A failure is rarely explained by the frame it happened on: glare that cuts a
 * path shows up in the frames before the detector gives up, and whether the
 * recovery was correct only shows in the frames after. The recorder therefore
 * keeps the most recent frames in memory and writes that whole window when
 * something triggers, plus the frames that follow it.
 *
 * Recording is off until [arm] is called. While disarmed the recorder costs the
 * caller nothing beyond an [isArmed] read, which matters because the caller is
 * a fixed-rate image pipeline.
 *
 * Frames are held by reference, never copied: the detector already owns a
 * private copy of each frame because the DJI callback array is reused, so the
 * ring adds retention but no extra copying. That also means a caller must not
 * mutate a frame after offering it.
 */
internal class TapeCaptureRecorder(
    root: File,
    private val log: (String) -> Unit,
    private val leadingFrameCount: Int = DEFAULT_LEADING_FRAME_COUNT,
    private val trailingFrameCount: Int = DEFAULT_TRAILING_FRAME_COUNT,
    private val store: TapeCaptureStore = TapeCaptureStore(root),
    private val writer: Executor = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "LiteTapeCapture").apply { isDaemon = true }
    },
) {
    init {
        require(leadingFrameCount > 0) { "leadingFrameCount must be > 0" }
        require(trailingFrameCount >= 0) { "trailingFrameCount must be >= 0" }
    }

    /**
     * Read by the detector once per frame before it does any capture work, so
     * it is the flag that keeps recording free while switched off.
     */
    @Volatile var isArmed: Boolean = false
        private set

    private val leadingFrames = ArrayDeque<TapeCapture>()
    private var pendingTrailingFrames = 0
    private var pendingReason = ""
    private var nextSequence = UNSEEDED_SEQUENCE
    private var savedCaptureCount = 0

    /** Captures written since the recorder was armed; shown to the operator. */
    val capturedFrameCount: Int
        @Synchronized get() = savedCaptureCount

    @Synchronized
    fun arm() {
        if (isArmed) return
        // Sequence numbers name the capture directories, so a restarted app must
        // continue the store's numbering instead of overwriting the previous
        // session's evidence with a second run of 000000000.
        if (nextSequence == UNSEEDED_SEQUENCE) nextSequence = store.nextSequence()
        isArmed = true
        log("tape capture armed existing=${store.captures().size} nextSeq=$nextSequence")
    }

    /**
     * Stops recording and drops the in-memory window. Frames already handed to
     * the writer still reach disk: they are evidence the operator asked for.
     */
    @Synchronized
    fun disarm() {
        if (!isArmed) return
        isArmed = false
        leadingFrames.clear()
        pendingTrailingFrames = 0
        pendingReason = ""
        log("tape capture disarmed captures=$savedCaptureCount bytes=${store.totalBytes()}")
    }

    /**
     * Offers one frame's evidence. While a trigger is still collecting trailing
     * frames the frame is written immediately; otherwise it joins the ring and
     * is written only if a later trigger reaches back for it.
     */
    @Synchronized
    fun offer(capture: TapeCapture) {
        if (!isArmed) return
        if (pendingTrailingFrames > 0) {
            // The detector triggers from inside the frame that lost the path, so
            // that frame is the next one offered: it is the event itself, and the
            // ones after it show whether the recovery was right.
            val suffix = if (pendingTrailingFrames == trailingFrameCount) "at" else "after"
            pendingTrailingFrames--
            write(capture, "$pendingReason-$suffix")
            return
        }
        leadingFrames.addLast(capture)
        while (leadingFrames.size > leadingFrameCount) leadingFrames.removeFirst()
    }

    /**
     * Marks the current moment as worth keeping. The buffered window is written
     * at once and the next [trailingFrameCount] offered frames follow it.
     *
     * A trigger while another is still collecting extends that one instead of
     * starting a second: a path lost over several frames is one event, and
     * splitting it would interleave two directories over the same moment.
     */
    @Synchronized
    fun trigger(reason: String) {
        if (!isArmed) return
        if (pendingTrailingFrames > 0) {
            pendingTrailingFrames = trailingFrameCount
            return
        }
        pendingReason = reason
        while (leadingFrames.isNotEmpty()) {
            write(leadingFrames.removeFirst(), "$reason-before")
        }
        pendingTrailingFrames = trailingFrameCount
    }

    private fun write(capture: TapeCapture, reason: String) {
        val sequence = nextSequence++
        savedCaptureCount++
        writer.execute {
            runCatching { store.save(capture, sequence, reason) }
                .onFailure { log("tape capture write failed seq=$sequence: ${it.message}") }
        }
    }

    internal companion object {
        /**
         * Four frames at the detector's 4 Hz intake is one second of run-up,
         * which covers a glare cut and the frames that led into it without
         * holding several full-resolution frames longer than necessary.
         */
        const val DEFAULT_LEADING_FRAME_COUNT = 4
        const val DEFAULT_TRAILING_FRAME_COUNT = 4
        private const val UNSEEDED_SEQUENCE = -1L
    }
}
