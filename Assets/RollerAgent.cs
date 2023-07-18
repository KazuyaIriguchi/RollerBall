using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;

public class RollerAgent : Agent
{
    // TargetのTransform
    public Transform target;
    // RollerAgentのRigidBody
    Rigidbody rBody;

    // ゲームオブジェクト生成時に呼ばれる
    public override void Initialize()
    {
        // RollerAgentnoRigidBodyの参照の取得
        this.rBody = GetComponent<Rigidbody>();
    }

    // エピソード開始時に呼ばれる
    public override void OnEpisodeBegin()
    {
        // RollerAgentが床から落下しているとき
        if (this.transform.localPosition.y < 0)
        {
            // RollerAgentの位置と速度をリセット
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0.0f, 0.5f, 0.0f);
        }

        // Targetの位置のリセット
        target.localPosition = new Vector3(
            Random.value * 8 - 4, 0.5f, Random.value * 8 - 4);
    }

    // 観察取得時に呼ばれる
    public override void CollectObservations(VectorSensor sensor)
    {
        // TargetのX座標
        sensor.AddObservation(target.localPosition.x);
        // TargetのZ座標
        sensor.AddObservation(target.localPosition.z);
        // RollerAgentのX座標
        sensor.AddObservation(this.transform.localPosition.x);
        // RollerAgentのZ座標
        sensor.AddObservation(this.transform.localPosition.z);
        // RollerAgentのX速度
        sensor.AddObservation(rBody.velocity.x);
        // RollerAgentのZ速度
        sensor.AddObservation(rBody.velocity.z);
    }

    // 行動決定時に呼ばれる
    public override void OnActionReceived(ActionBuffers actions)
    {
        // RollerAgentに力を加える
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.z = actions.ContinuousActions[1];
        rBody.AddForce(controlSignal * 10);

        // RollerAgentがTargetの位置にたどり着いたとき
        float distanceToTarget = Vector3.Distance(
            this.transform.localPosition, target.localPosition);
        if (distanceToTarget < 1.42f)
        {
            AddReward(1.0f);
            EndEpisode();
        }

        // RollerAgentが床から落下したとき
        if (this.transform.localPosition.y < 0)
        {
            EndEpisode();
        }
    }

    // ヒューリスティックモードの行動決定時に呼ばれる
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }
}
