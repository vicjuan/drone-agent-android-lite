package com.durendal.droneagent.lite

import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

/**
 * Append-only monotonic trace used to split one real flight into software and
 * aircraft-response timing. Callers only format one short line; disk I/O stays
 * on a dedicated thread and is flushed at least once per second at 20 Hz.
 */
internal class FlightProfiler(
    file: File,
    private val clock: () -> Long = System::nanoTime,
) : AutoCloseable {

    val path: String = file.absolutePath

    private val startedAtNanos = clock()
    private var closed = false
    private val writerExecutor = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "LiteFlightProfiler").apply { isDaemon = true }
    }
    private val writer = file.apply {
        parentFile?.mkdirs()
    }.bufferedWriter()
    private var lastFlushedAtNanos = 0L

    init {
        writer.write("elapsedMs\tmonotonicNanos\tevent\tframeNanos\tdurationMs\tdetails\n")
        writer.flush()
    }

    @Synchronized
    fun record(
        event: String,
        atNanos: Long = clock(),
        frameNanos: Long = 0L,
        durationNanos: Long = 0L,
        details: String = "",
    ) {
        if (closed) return
        val elapsedMillis = (atNanos - startedAtNanos).coerceAtLeast(0L) / NANOS_PER_MILLISECOND
        val durationMillis = durationNanos.coerceAtLeast(0L) / NANOS_PER_MILLISECOND
        val line = buildString(192) {
            append(elapsedMillis)
            append('\t').append(atNanos)
            append('\t').append(sanitize(event))
            append('\t').append(frameNanos)
            append('\t').append(durationMillis)
            append('\t').append(sanitize(details))
            append('\n')
        }
        writerExecutor.execute {
            writer.write(line)
            val nowNanos = System.nanoTime()
            if (nowNanos - lastFlushedAtNanos >= FLUSH_INTERVAL_NANOS) {
                writer.flush()
                lastFlushedAtNanos = nowNanos
            }
        }
    }

    @Synchronized
    override fun close() {
        if (closed) return
        closed = true
        val closeTask = writerExecutor.submit {
            writer.flush()
            writer.close()
        }
        try {
            closeTask.get(CLOSE_TIMEOUT_SECONDS, TimeUnit.SECONDS)
        } catch (error: Throwable) {
            throw IllegalStateException("could not close profiling trace $path", error)
        } finally {
            writerExecutor.shutdownNow()
        }
    }

    private fun sanitize(value: String): String =
        value.replace('\t', ' ').replace('\n', ' ').replace('\r', ' ')

    private companion object {
        const val NANOS_PER_MILLISECOND = 1_000_000L
        const val FLUSH_INTERVAL_NANOS = 1_000_000_000L
        const val CLOSE_TIMEOUT_SECONDS = 2L
    }
}

internal fun profileDetails(vararg fields: Pair<String, Any?>): String =
    fields.joinToString(" ") { (name, value) -> "$name=${value ?: "null"}" }
