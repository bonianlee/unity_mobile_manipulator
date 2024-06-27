using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosHandPosMsg = RosMessageTypes.Assets.HandPosMsgMsg;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using Assimp;
using System;

public class HandPositionPublisher : MonoBehaviour
{
    HandController handController;
    [SerializeField] float publishMsgFreq = 0.05f;
    private ROSConnection ros;
    private string topicname = "ref_pos";
    private float timeElapsed;

    private RosHandPosMsg msg;
    // Start is called before the first frame update
    void Start()
    {
        handController = GetComponent<HandController>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosHandPosMsg>(topicname);
        msg = new RosHandPosMsg();
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMsgFreq)
        {
            // 手把相對大地座標
            Vector3 tmp_handle_pos = handController.GetHandPosition3DValue();
            UnityEngine.Quaternion tmp_handle_quat = handController.GetHandRotation3DValue();
            // HMD相對於大地座標
            Vector3 tmp_HMD_pos = handController.GetHMDPosition3DValue();
            UnityEngine.Quaternion tmp_HMD_quat = handController.GetHMDRotation3DValue();
            // 旋轉向量(HMD到手把，再轉到機器人的基底座標 軸角表示)
            Matrix3x3 ground2handle_right_hand_rotm = Quat2Rotm_inv(tmp_handle_quat);
            Matrix3x3 ground2HMD_rotm = Quat2Rotm(tmp_HMD_quat); // HMD 為左手系座標
            Matrix3x3 ground2HMD_right_hand_rotm = Quat2Rotm_inv(tmp_HMD_quat); // HMD 為右手系座標
            Vector3 tmp_rot = RobotBase2handle_angleAxis(ground2handle_right_hand_rotm, ground2HMD_right_hand_rotm);
            // 位置向量(HMD到手把，再旋轉到機器人基底座標)
            Vector3 tmp_posVec_HMD2handle = tmp_handle_pos - tmp_HMD_pos;
            Vector3 tmp_pos = RobotBase2handle_pos(tmp_posVec_HMD2handle, ground2HMD_rotm);

            // 代入msg
            SetHandPos(tmp_pos[0], tmp_pos[1], tmp_pos[2]);
            SetHandRot(tmp_rot);

            // debug
            // Vector3 angleAxis_show = angle * axis;
            // Debug.Log("HMD:" + tmp_HMD_pos + "\n");
            // Debug.Log("hand:" + tmp_handle_pos + "\n");

            Debug.Log("tmp_rot:" + tmp_rot);

            // publish
            ros.Publish(topicname, msg);
            timeElapsed = 0;
        }
    }

    private void SetHandPos(float linear_x, float linear_y, float linear_z)
    {
        // 左手系座標轉右手系座標，將 z 取反向
        msg.hand_pos[0] = linear_x;
        msg.hand_pos[1] = linear_y;
        msg.hand_pos[2] = linear_z;
    }

    private void SetHandRot(Vector3 tmp_rot)
    {
        msg.hand_rot[0] = tmp_rot[0];
        msg.hand_rot[1] = tmp_rot[1];
        msg.hand_rot[2] = tmp_rot[2];
    }

    private Matrix3x3 Quat2Rotm(UnityEngine.Quaternion quat)
    {
        // 四元數轉旋轉矩陣
        // 轉換關係的符號對照: a=w, b=x, c=y, d=z
        // UnityEngine.Quaternion 的符號對照: 0=x, 1=y, 2=z, 3=w
        Matrix3x3 rotm = new Matrix3x3();
        rotm[1, 1] = (float)(2 * Math.Pow(quat[3], 2) + 2 * Math.Pow(quat[0], 2) - 1);
        // rotm[1, 1] = (float)(1 - 2 * Math.Pow(quat[1], 2) - 2 * Math.Pow(quat[2], 2));
        rotm[1, 2] = (float)(2 * quat[0] * quat[1] - 2 * quat[3] * quat[2]);
        rotm[1, 3] = (float)(2 * quat[0] * quat[2] + 2 * quat[3] * quat[1]);
        rotm[2, 1] = (float)(2 * quat[0] * quat[1] + 2 * quat[3] * quat[2]);
        rotm[2, 2] = (float)(2 * Math.Pow(quat[3], 2) + 2 * Math.Pow(quat[1], 2) - 1);
        // rotm[2, 2] = (float)(1 - 2 * Math.Pow(quat[0], 2) - 2 * Math.Pow(quat[2], 2));
        rotm[2, 3] = (float)(2 * quat[1] * quat[2] - 2 * quat[3] * quat[0]);
        rotm[3, 1] = (float)(2 * quat[0] * quat[2] - 2 * quat[3] * quat[1]);
        rotm[3, 2] = (float)(2 * quat[1] * quat[2] + 2 * quat[3] * quat[0]);
        rotm[3, 3] = (float)(2 * Math.Pow(quat[3], 2) + 2 * Math.Pow(quat[2], 2) - 1);
        // rotm[3, 3] = (float)(1 - 2 * Math.Pow(quat[0], 2) - 2 * Math.Pow(quat[1], 2));
        return rotm;
    }

    private Matrix3x3 Quat2Rotm_inv(UnityEngine.Quaternion quat)
    {
        // Unity 中採用左手系座標，因此需轉換到常用的右手系座標 (四元數轉法:w,z不變，x,y取反向)
        // 轉換關係的符號對照: a=w, b=x, c=y, d=z
        // UnityEngine.Quaternion 的符號對照: 0=x, 1=y, 2=z, 3=w
        // UnityEngine.Quaternion quat_inv = quat;
        UnityEngine.Quaternion quat_inv = new UnityEngine.Quaternion(-quat.x, -quat.y, quat.z, quat.w);
        // 轉到右手系 (x,y 取反)
        // for (int i = 0; i < 2; i++)
        //     quat_inv[i] = -quat_inv[i];
        Matrix3x3 rotm = new Matrix3x3();
        rotm[1, 1] = (float)(2 * Math.Pow(quat_inv[3], 2) + 2 * Math.Pow(quat_inv[0], 2) - 1);
        // rotm[1, 1] = (float)(1 - 2 * Math.Pow(quat_inv[1], 2) - 2 * Math.Pow(quat_inv[2], 2));
        rotm[1, 2] = (float)(2 * quat_inv[0] * quat_inv[1] - 2 * quat_inv[3] * quat_inv[2]);
        rotm[1, 3] = (float)(2 * quat_inv[0] * quat_inv[2] + 2 * quat_inv[3] * quat_inv[1]);
        rotm[2, 1] = (float)(2 * quat_inv[0] * quat_inv[1] + 2 * quat_inv[3] * quat_inv[2]);
        rotm[2, 2] = (float)(2 * Math.Pow(quat_inv[3], 2) + 2 * Math.Pow(quat_inv[1], 2) - 1);
        // rotm[2, 2] = (float)(1 - 2 * Math.Pow(quat_inv[0], 2) - 2 * Math.Pow(quat_inv[2], 2));
        rotm[2, 3] = (float)(2 * quat_inv[1] * quat_inv[2] - 2 * quat_inv[3] * quat_inv[0]);
        rotm[3, 1] = (float)(2 * quat_inv[0] * quat_inv[2] - 2 * quat_inv[3] * quat_inv[1]);
        rotm[3, 2] = (float)(2 * quat_inv[1] * quat_inv[2] + 2 * quat_inv[3] * quat_inv[0]);
        rotm[3, 3] = (float)(2 * Math.Pow(quat_inv[3], 2) + 2 * Math.Pow(quat_inv[2], 2) - 1);
        // rotm[3, 3] = (float)(1 - 2 * Math.Pow(quat_inv[0], 2) - 2 * Math.Pow(quat_inv[1], 2));
        return rotm;
    }

    private Vector3 RobotBase2handle_angleAxis(Matrix3x3 ground2handle_right_hand_rotm, Matrix3x3 ground2HMD_right_hand_rotm)
    {
        // ground2handle_right_hand_rotm.Inverse();
        ground2HMD_right_hand_rotm.Inverse();
        Matrix3x3 HMD2ground_right_hand_rotm = ground2HMD_right_hand_rotm;
        Matrix3x3 HMD2handle_rotm = ground2handle_right_hand_rotm * HMD2ground_right_hand_rotm; // the operator "*" definition please see its API
        Matrix3x3 robotBase2HMD_rotm = new Matrix3x3(0, 0, -1, -1, 0, 0, 0, 1, 0);
        Matrix3x3 robotBase2handle_rotm = HMD2handle_rotm * robotBase2HMD_rotm;
        float trace = robotBase2handle_rotm[1, 1] + robotBase2handle_rotm[2, 2] + robotBase2handle_rotm[3, 3];
        float angle = (float)(Math.Acos((trace - 1) / 2));
        Vector3 axis = new Vector3();
        axis[0] = (float)((1 / (2 * Math.Sin(angle))) * (robotBase2handle_rotm[3, 2] - robotBase2handle_rotm[2, 3]));
        axis[1] = (float)((1 / (2 * Math.Sin(angle))) * (robotBase2handle_rotm[1, 3] - robotBase2handle_rotm[3, 1]));
        axis[2] = (float)((1 / (2 * Math.Sin(angle))) * (robotBase2handle_rotm[2, 1] - robotBase2handle_rotm[1, 2]));
        return angle * axis;
    }

    private Vector3 RobotBase2handle_pos(Vector3 tmp_posVec_HMD2handle, Matrix3x3 ground2HMD_rotm)
    {
        ground2HMD_rotm.Inverse();
        Matrix3x3 HMD2ground_rotm = ground2HMD_rotm;
        // 原本 HMD2handle 的向量是相對於大地座標，現在轉至相對於 HMD 座標系
        Vector3 tmp_pos_HMD2handle = Vector3.zero;
        tmp_pos_HMD2handle[0] = HMD2ground_rotm[1, 1] * tmp_posVec_HMD2handle[0] + HMD2ground_rotm[1, 2] * tmp_posVec_HMD2handle[1] + HMD2ground_rotm[1, 3] * tmp_posVec_HMD2handle[2];
        tmp_pos_HMD2handle[1] = HMD2ground_rotm[2, 1] * tmp_posVec_HMD2handle[0] + HMD2ground_rotm[2, 2] * tmp_posVec_HMD2handle[1] + HMD2ground_rotm[2, 3] * tmp_posVec_HMD2handle[2];
        tmp_pos_HMD2handle[2] = HMD2ground_rotm[3, 1] * tmp_posVec_HMD2handle[0] + HMD2ground_rotm[3, 2] * tmp_posVec_HMD2handle[1] + HMD2ground_rotm[3, 3] * tmp_posVec_HMD2handle[2];
        // 將左手系座標轉至右手系座標
        Vector3 tf2RightHandFrame = Vector3.zero;
        for (int i = 0; i < 2; i++)
            tf2RightHandFrame[i] = tmp_pos_HMD2handle[i];
        tf2RightHandFrame[2] = -tmp_pos_HMD2handle[2];
        // 將基底座標由 HMD 轉至機器人基底座標
        Matrix3x3 robotBase2HMD_rotm = new Matrix3x3(0, 0, -1, -1, 0, 0, 0, 1, 0);
        Vector3 tmp_result = Vector3.zero;
        tmp_result[0] = robotBase2HMD_rotm[1, 1] * tf2RightHandFrame[0] + robotBase2HMD_rotm[1, 2] * tf2RightHandFrame[1] + robotBase2HMD_rotm[1, 3] * tf2RightHandFrame[2];
        tmp_result[1] = robotBase2HMD_rotm[2, 1] * tf2RightHandFrame[0] + robotBase2HMD_rotm[2, 2] * tf2RightHandFrame[1] + robotBase2HMD_rotm[2, 3] * tf2RightHandFrame[2];
        tmp_result[2] = robotBase2HMD_rotm[3, 1] * tf2RightHandFrame[0] + robotBase2HMD_rotm[3, 2] * tf2RightHandFrame[1] + robotBase2HMD_rotm[3, 3] * tf2RightHandFrame[2];
        return tmp_result;
    }
}