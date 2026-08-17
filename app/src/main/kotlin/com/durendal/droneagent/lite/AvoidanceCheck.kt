package com.durendal.droneagent.lite

import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.ObstacleAvoidanceType

/**
 * Changes the aircraft's own obstacle-avoidance mode and verifies the read-back.
 *
 * Normal horizontal control requires [ObstacleAvoidanceType.BRAKE]. Tape tracking
 * can temporarily request [ObstacleAvoidanceType.CLOSE] because the Mini 4 Pro's
 * firmware interprets the floor as a 360-degree obstacle at this flight height.
 */
class AvoidanceCheck {

    data class Status(
        val type: ObstacleAvoidanceType? = null,
        val confirmed: Boolean = false,
        val detail: String? = null,
    ) {
        val brakeConfirmed: Boolean
            get() = confirmed && type == ObstacleAvoidanceType.BRAKE

        val closedConfirmed: Boolean
            get() = confirmed && type == ObstacleAvoidanceType.CLOSE

        val summary: String
            get() = "avoidance=${type?.name ?: "unverified"} confirmed=$confirmed"

        val warning: String?
            get() = when {
                detail != null -> "避障模式設定失敗：$detail"
                type == ObstacleAvoidanceType.CLOSE -> "飛機避障目前為關閉"
                type == ObstacleAvoidanceType.BYPASS -> "飛機避障目前為繞行"
                type == ObstacleAvoidanceType.BRAKE -> "飛機 BRAKE 避障尚未確認"
                else -> "尚未確認飛機避障模式"
            }
    }

    fun ensureBrake(onStatus: (Status) -> Unit) =
        ensureMode(ObstacleAvoidanceType.BRAKE, onStatus)

    fun ensureClosed(onStatus: (Status) -> Unit) =
        ensureMode(ObstacleAvoidanceType.CLOSE, onStatus)

    private fun ensureMode(
        requiredType: ObstacleAvoidanceType,
        onStatus: (Status) -> Unit,
    ) {
        val perception = PerceptionManager.getInstance()
        perception.getObstacleAvoidanceType(
            object : CommonCallbacks.CompletionCallbackWithParam<ObstacleAvoidanceType> {
                override fun onSuccess(type: ObstacleAvoidanceType) {
                    if (type == requiredType) {
                        onStatus(Status(type, confirmed = true))
                    } else {
                        setMode(requiredType, type, onStatus)
                    }
                }

                override fun onFailure(error: IDJIError) =
                    onStatus(Status(detail = error.description() ?: error.toString()))
            },
        )
    }

    private fun setMode(
        requiredType: ObstacleAvoidanceType,
        previousType: ObstacleAvoidanceType,
        onStatus: (Status) -> Unit,
    ) {
        val perception = PerceptionManager.getInstance()
        perception.setObstacleAvoidanceType(
            requiredType,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() = verifyMode(requiredType, onStatus)

                override fun onFailure(error: IDJIError) = onStatus(
                    Status(previousType, detail = error.description() ?: error.toString()),
                )
            },
        )
    }

    private fun verifyMode(
        requiredType: ObstacleAvoidanceType,
        onStatus: (Status) -> Unit,
    ) {
        val perception = PerceptionManager.getInstance()
        perception.getObstacleAvoidanceType(
            object : CommonCallbacks.CompletionCallbackWithParam<ObstacleAvoidanceType> {
                override fun onSuccess(type: ObstacleAvoidanceType) =
                    onStatus(Status(type, confirmed = type == requiredType))

                override fun onFailure(error: IDJIError) =
                    onStatus(Status(detail = error.description() ?: error.toString()))
            },
        )
    }
}
