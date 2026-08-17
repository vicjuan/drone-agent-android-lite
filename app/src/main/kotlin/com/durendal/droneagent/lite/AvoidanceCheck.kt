package com.durendal.droneagent.lite

import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.data.ObstacleAvoidanceType

/**
 * Puts a connected aircraft in BRAKE mode and verifies the read-back.
 *
 * Horizontal app control remains locked until [Status.brakeConfirmed] is true.
 * A failed get, failed set, or mismatched read-back is therefore safe by default.
 */
class AvoidanceCheck {

    data class Status(
        val type: ObstacleAvoidanceType? = null,
        val brakeConfirmed: Boolean = false,
        val detail: String? = null,
    ) {
        val summary: String
            get() = "avoidance=${type?.name ?: "unverified"} brakeConfirmed=$brakeConfirmed"

        val warning: String?
            get() = when {
                brakeConfirmed -> null
                detail != null -> "避障 BRAKE 設定失敗：$detail"
                type == ObstacleAvoidanceType.CLOSE -> "飛機避障仍為關閉"
                type == ObstacleAvoidanceType.BYPASS -> "飛機避障仍為繞行"
                else -> "尚未確認飛機 BRAKE 避障模式"
            }
    }

    fun ensureBrake(onStatus: (Status) -> Unit) {
        val perception = PerceptionManager.getInstance()
        perception.getObstacleAvoidanceType(
            object : CommonCallbacks.CompletionCallbackWithParam<ObstacleAvoidanceType> {
                override fun onSuccess(type: ObstacleAvoidanceType) {
                    if (type == ObstacleAvoidanceType.BRAKE) {
                        onStatus(Status(type, brakeConfirmed = true))
                    } else {
                        setBrake(type, onStatus)
                    }
                }

                override fun onFailure(error: IDJIError) =
                    onStatus(Status(detail = error.description() ?: error.toString()))
            },
        )
    }

    private fun setBrake(
        previousType: ObstacleAvoidanceType,
        onStatus: (Status) -> Unit,
    ) {
        val perception = PerceptionManager.getInstance()
        perception.setObstacleAvoidanceType(
            ObstacleAvoidanceType.BRAKE,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() = verifyBrake(onStatus)

                override fun onFailure(error: IDJIError) = onStatus(
                    Status(previousType, detail = error.description() ?: error.toString()),
                )
            },
        )
    }

    private fun verifyBrake(onStatus: (Status) -> Unit) {
        val perception = PerceptionManager.getInstance()
        perception.getObstacleAvoidanceType(
            object : CommonCallbacks.CompletionCallbackWithParam<ObstacleAvoidanceType> {
                override fun onSuccess(type: ObstacleAvoidanceType) =
                    onStatus(Status(type, brakeConfirmed = type == ObstacleAvoidanceType.BRAKE))

                override fun onFailure(error: IDJIError) =
                    onStatus(Status(detail = error.description() ?: error.toString()))
            },
        )
    }
}
