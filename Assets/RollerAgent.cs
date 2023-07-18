using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;

public class RollerAgent : Agent
{
    // Target��Transform
    public Transform target;
    // RollerAgent��RigidBody
    Rigidbody rBody;

    // �Q�[���I�u�W�F�N�g�������ɌĂ΂��
    public override void Initialize()
    {
        // RollerAgentnoRigidBody�̎Q�Ƃ̎擾
        this.rBody = GetComponent<Rigidbody>();
    }

    // �G�s�\�[�h�J�n���ɌĂ΂��
    public override void OnEpisodeBegin()
    {
        // RollerAgent�������痎�����Ă���Ƃ�
        if (this.transform.localPosition.y < 0)
        {
            // RollerAgent�̈ʒu�Ƒ��x�����Z�b�g
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0.0f, 0.5f, 0.0f);
        }

        // Target�̈ʒu�̃��Z�b�g
        target.localPosition = new Vector3(
            Random.value * 8 - 4, 0.5f, Random.value * 8 - 4);
    }

    // �ώ@�擾���ɌĂ΂��
    public override void CollectObservations(VectorSensor sensor)
    {
        // Target��X���W
        sensor.AddObservation(target.localPosition.x);
        // Target��Z���W
        sensor.AddObservation(target.localPosition.z);
        // RollerAgent��X���W
        sensor.AddObservation(this.transform.localPosition.x);
        // RollerAgent��Z���W
        sensor.AddObservation(this.transform.localPosition.z);
        // RollerAgent��X���x
        sensor.AddObservation(rBody.velocity.x);
        // RollerAgent��Z���x
        sensor.AddObservation(rBody.velocity.z);
    }

    // �s�����莞�ɌĂ΂��
    public override void OnActionReceived(ActionBuffers actions)
    {
        // RollerAgent�ɗ͂�������
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.z = actions.ContinuousActions[1];
        rBody.AddForce(controlSignal * 10);

        // RollerAgent��Target�̈ʒu�ɂ��ǂ蒅�����Ƃ�
        float distanceToTarget = Vector3.Distance(
            this.transform.localPosition, target.localPosition);
        if (distanceToTarget < 1.42f)
        {
            AddReward(1.0f);
            EndEpisode();
        }

        // RollerAgent�������痎�������Ƃ�
        if (this.transform.localPosition.y < 0)
        {
            EndEpisode();
        }
    }

    // �q���[���X�e�B�b�N���[�h�̍s�����莞�ɌĂ΂��
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Horizontal");
        continuousActions[1] = Input.GetAxis("Vertical");
    }
}
