using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosHandPosMsg = RosMessageTypes.Assets.HandPosMsgMsg;
using RosOdom = RosMessageTypes.Nav.OdometryMsg;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using Assimp;
using System;

public class HandPositionPublisher : MonoBehaviour
{
    HandController handController;
    // PlatformMoving platformState;
    [SerializeField] float publishMsgFreq = 0.0166f;
    private ROSConnection ros;
    private string topicname = "ref_pos";
    private float timeElapsed;
    private Vector3 position = Vector3.zero;
    private UnityEngine.Quaternion rotation = UnityEngine.Quaternion.identity;
    Matrix3x3 R_he;
    Vector3 tmp_rot_w_init;

    private RosHandPosMsg msg;
    // Start is called before the first frame update
    void Start()
    {
        handController = GetComponent<HandController>();
        // platformState = GetComponent<PlatformMoving>();

        // For 轉換手把旋轉軸
        UnityEngine.Quaternion tmp_handle_quat = handController.GetHandRotation3DValue();
        UnityEngine.Quaternion tmp_HMD_quat = handController.GetHMDRotation3DValue();
        Matrix3x3 ground2handle_right_hand_rotm = Quat2Rotm_inv(tmp_handle_quat);
        Matrix3x3 ground2HMD_right_hand_rotm = Quat2Rotm_inv(tmp_HMD_quat); // HMD 為右手系座標
        Matrix3x3 R_gh = wholeBodyGround2handle_rotm(ground2handle_right_hand_rotm, ground2HMD_right_hand_rotm);
        Matrix3x3 R_ge = new Matrix3x3(0, 0, 1, 1, 0, 0, 0, 1, 0);
        Matrix3x3 R_hg = R_gh;
        R_hg.Inverse();
        R_he = R_ge * R_hg;
        // Matrix3x3 R_ge_handle = R_he * R_gh;
        // tmp_rot_w_init = rotm2angleAxis(R_ge_handle);


        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RosHandPosMsg>(topicname);
        msg = new RosHandPosMsg();
        ros.Subscribe<RosOdom>("odom", odomChange);
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
            Matrix3x3 R_mh = m2handle_rotm(ground2handle_right_hand_rotm, ground2HMD_right_hand_rotm);
            Matrix3x3 R_gh = wholeBodyGround2handle_rotm(ground2handle_right_hand_rotm, ground2HMD_right_hand_rotm);
            // ----
            Matrix3x3 R_ge_handle = R_he * R_gh;
            Vector3 tmp_rot_w = rotm2angleAxis(R_ge_handle);
            // ----
            Vector3 tmp_rot_m = rotm2angleAxis(R_mh);

            // Vector3 tmp_rot_w = rotm2angleAxis(R_gh);
            // 位置向量(HMD到手把，再旋轉到機器人基底座標)
            Vector3 tmp_posVec_HMD2handle = tmp_handle_pos - tmp_HMD_pos;
            Vector3 tmp_pos_m = m2handle_pos(tmp_posVec_HMD2handle, ground2HMD_rotm);
            Vector3 tmp_pos_w = wholeBodyGround2handle_pos(tmp_posVec_HMD2handle, ground2HMD_rotm);

            // 代入msg
            SetHandPos(tmp_pos_w, tmp_pos_m);
            SetHandRot(tmp_rot_w, tmp_rot_m);

            // debug
            // Vector3 angleAxis_show = angle * axis;
            // Debug.Log("HMD:" + tmp_HMD_pos + "\n");
            // Debug.Log("hand:" + tmp_rot_w + "\n");
            

            Debug.Log("tmp_pos:" + tmp_pos_w + "\n");
            Debug.Log("tmp_rot:" + tmp_rot_w + "\n");

            // publish
            ros.Publish(topicname, msg);
            timeElapsed = 0;
        }
    }

    private void SetHandPos(Vector3 tmp_pos_w, Vector3 tmp_pos_m)
    {
        msg.hand_pos_w[0] = tmp_pos_w[0];
        msg.hand_pos_w[1] = tmp_pos_w[1];
        msg.hand_pos_w[2] = tmp_pos_w[2];

        msg.hand_pos_m[0] = tmp_pos_m[0];
        msg.hand_pos_m[1] = tmp_pos_m[1];
        msg.hand_pos_m[2] = tmp_pos_m[2];
    }

    private void SetHandRot(Vector3 tmp_rot_w, Vector3 tmp_rot_m)
    {
        msg.hand_rot_w[0] = tmp_rot_w[0];
        msg.hand_rot_w[1] = tmp_rot_w[1];
        msg.hand_rot_w[2] = tmp_rot_w[2];

        msg.hand_rot_m[0] = tmp_rot_m[0];
        msg.hand_rot_m[1] = tmp_rot_m[1];
        msg.hand_rot_m[2] = tmp_rot_m[2];
    }

    private Matrix3x3 Quat2Rotm(UnityEngine.Quaternion quat)
    {
        // 四元數轉旋轉矩陣
        // 轉換關係的符號對照: a=w, b=x, c=y, d=z
        // UnityEngine.Quaternion 的符號對照: 0=x, 1=y, 2=z, 3=w
        Matrix3x3 rotm = new Matrix3x3();
        // rotm[1, 1] = (float)(2 * Math.Pow(quat[3], 2) + 2 * Math.Pow(quat[0], 2) - 1);
        rotm[1, 1] = (float)(2 * quat[3] * quat[3] + 2 * quat[0] * quat[0] - 1);
        rotm[1, 2] = (float)(2 * quat[0] * quat[1] - 2 * quat[3] * quat[2]);
        rotm[1, 3] = (float)(2 * quat[0] * quat[2] + 2 * quat[3] * quat[1]);
        rotm[2, 1] = (float)(2 * quat[0] * quat[1] + 2 * quat[3] * quat[2]);
        // rotm[2, 2] = (float)(2 * Math.Pow(quat[3], 2) + 2 * Math.Pow(quat[1], 2) - 1);
        rotm[2, 2] = (float)(2 * quat[3] * quat[3] + 2 * quat[1] * quat[1] - 1);
        rotm[2, 3] = (float)(2 * quat[1] * quat[2] - 2 * quat[3] * quat[0]);
        rotm[3, 1] = (float)(2 * quat[0] * quat[2] - 2 * quat[3] * quat[1]);
        rotm[3, 2] = (float)(2 * quat[1] * quat[2] + 2 * quat[3] * quat[0]);
        // rotm[3, 3] = (float)(2 * Math.Pow(quat[3], 2) + 2 * Math.Pow(quat[2], 2) - 1);
        rotm[3, 3] = (float)(2 * quat[3] * quat[3] + 2 * quat[2] * quat[2] - 1);
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
        // rotm[1, 1] = (float)(2 * Math.Pow(quat_inv[3], 2) + 2 * Math.Pow(quat_inv[0], 2) - 1);
        rotm[1, 1] = (float)(2 * quat_inv[3] * quat_inv[3] + 2 * quat_inv[0] * quat_inv[0] - 1);
        rotm[1, 2] = (float)(2 * quat_inv[0] * quat_inv[1] - 2 * quat_inv[3] * quat_inv[2]);
        rotm[1, 3] = (float)(2 * quat_inv[0] * quat_inv[2] + 2 * quat_inv[3] * quat_inv[1]);
        rotm[2, 1] = (float)(2 * quat_inv[0] * quat_inv[1] + 2 * quat_inv[3] * quat_inv[2]);
        // rotm[2, 2] = (float)(2 * Math.Pow(quat_inv[3], 2) + 2 * Math.Pow(quat_inv[1], 2) - 1);
        rotm[2, 2] = (float)(2 * quat_inv[3] * quat_inv[3] + 2 * quat_inv[1] * quat_inv[1] - 1);
        rotm[2, 3] = (float)(2 * quat_inv[1] * quat_inv[2] - 2 * quat_inv[3] * quat_inv[0]);
        rotm[3, 1] = (float)(2 * quat_inv[0] * quat_inv[2] - 2 * quat_inv[3] * quat_inv[1]);
        rotm[3, 2] = (float)(2 * quat_inv[1] * quat_inv[2] + 2 * quat_inv[3] * quat_inv[0]);
        // rotm[3, 3] = (float)(2 * Math.Pow(quat_inv[3], 2) + 2 * Math.Pow(quat_inv[2], 2) - 1);
        rotm[3, 3] = (float)(2 * quat_inv[3] * quat_inv[3] + 2 * quat_inv[2] * quat_inv[2] - 1);
        return rotm;
    }

    void odomChange(RosOdom odomMsg)
    {
        var pos = getPosition(odomMsg);
        position = new Vector3(-pos.y, 0, pos.x);
        var rot = getRotation(odomMsg);
        rotation = new UnityEngine.Quaternion(0, -rot.z, 0, rot.w);
    }

    Vector3 getPosition(RosOdom odomMsg)
    {
        return new Vector3((float)odomMsg.pose.pose.position.x, (float)odomMsg.pose.pose.position.y, (float)odomMsg.pose.pose.position.z);
    }

    UnityEngine.Quaternion getRotation(RosOdom odomMsg)
    {
        return new UnityEngine.Quaternion(
            (float)odomMsg.pose.pose.orientation.x,
            (float)odomMsg.pose.pose.orientation.y,
            (float)odomMsg.pose.pose.orientation.z,
            (float)odomMsg.pose.pose.orientation.w
        );
    }

    private UnityEngine.Quaternion rotm2quaternion(Matrix3x3 rotm)
    {
        // 嘗試轉換手把的初始旋轉軸，使其與末端效應點的初始旋轉軸一致
        UnityEngine.Quaternion quaternion_gh = new UnityEngine.Quaternion((float)(0.5 * Math.Sqrt(1 + rotm[1, 1] - rotm[2, 2] - rotm[3, 3])), (float)(0.5 * Math.Sqrt(1 - rotm[1, 1] + rotm[2, 2] - rotm[3, 3])), (float)(0.5 * Math.Sqrt(1 - rotm[1, 1] - rotm[2, 2] + rotm[3, 3])), (float)(0.5 * Math.Sqrt(1 + rotm[1, 1] + rotm[2, 2] + rotm[3, 3])));
        if (Math.Sign(quaternion_gh[3]) * Math.Sign(rotm[3, 2] - rotm[2, 3]) < 0)
            quaternion_gh[0] = -quaternion_gh[0];
        if (Math.Sign(quaternion_gh[3]) * Math.Sign(rotm[1, 3] - rotm[3, 1]) < 0)
            quaternion_gh[1] = -quaternion_gh[1];
        if (Math.Sign(quaternion_gh[3]) * Math.Sign(rotm[2, 1] - rotm[1, 2]) < 0)
            quaternion_gh[2] = -quaternion_gh[2];
        return quaternion_gh;
    }

    private UnityEngine.Quaternion Calculate_adjustment_quat(Matrix3x3 curr_rotm)
    {
        Vector3 target_axis = new Vector3(0.5774f, 0.5774f, 0.5774f);

        //
        float trace = curr_rotm[1, 1] + curr_rotm[2, 2] + curr_rotm[3, 3];
        float angle = (float)(Math.Acos((trace - 1) / 2));
        Vector3 curr_axis = new Vector3();
        curr_axis[0] = (float)((1 / (2 * Math.Sin(angle))) * (curr_rotm[3, 2] - curr_rotm[2, 3]));
        curr_axis[1] = (float)((1 / (2 * Math.Sin(angle))) * (curr_rotm[1, 3] - curr_rotm[3, 1]));
        curr_axis[2] = (float)((1 / (2 * Math.Sin(angle))) * (curr_rotm[2, 1] - curr_rotm[1, 2]));

        //
        UnityEngine.Quaternion currentRotation = UnityEngine.Quaternion.LookRotation(curr_axis);
        UnityEngine.Quaternion desiredRotation = UnityEngine.Quaternion.LookRotation(target_axis);

        return UnityEngine.Quaternion.Inverse(currentRotation) * desiredRotation;
    }

    private Matrix3x3 m2handle_rotm(Matrix3x3 ground2handle_right_hand_rotm, Matrix3x3 ground2HMD_right_hand_rotm)
    {
        // ground2handle_right_hand_rotm.Inverse();
        ground2HMD_right_hand_rotm.Inverse();
        Matrix3x3 HMD2ground_right_hand_rotm = ground2HMD_right_hand_rotm;
        Matrix3x3 HMD2handle_rotm = ground2handle_right_hand_rotm * HMD2ground_right_hand_rotm; // the operator "*" definition please see its API
        Matrix3x3 robotBase2HMD_rotm = new Matrix3x3(0, 0, -1, -1, 0, 0, 0, 1, 0);
        Matrix3x3 robotBase2handle_rotm = HMD2handle_rotm * robotBase2HMD_rotm;
        return robotBase2handle_rotm;
    }
    
    private Matrix3x3 wholeBodyGround2handle_rotm(Matrix3x3 ground2handle_right_hand_rotm, Matrix3x3 ground2HMD_right_hand_rotm)
    {
        // ground2handle_right_hand_rotm.Inverse();
        ground2HMD_right_hand_rotm.Inverse();
        Matrix3x3 HMD2ground_right_hand_rotm = ground2HMD_right_hand_rotm;
        Matrix3x3 HMD2handle_rotm = ground2handle_right_hand_rotm * HMD2ground_right_hand_rotm; // the operator "*" definition please see its API
        Matrix3x3 robotBase2HMD_rotm = new Matrix3x3(0, 0, -1, -1, 0, 0, 0, 1, 0);
        Matrix3x3 robotBase2handle_rotm = HMD2handle_rotm * robotBase2HMD_rotm;
        double platform_yaw = Math.Atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y), 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z));
        Matrix3x3 wholeBodyGround2robotBase_rotm = new Matrix3x3((float)Math.Cos(platform_yaw), (float)-Math.Sin(platform_yaw), 0, (float)Math.Sin(platform_yaw), (float)Math.Cos(platform_yaw), 0, 0, 0, 1);
        Matrix3x3 rotm = robotBase2handle_rotm * wholeBodyGround2robotBase_rotm;
        return rotm;
    }

    private Vector3 rotm2angleAxis(Matrix3x3 rotm)
    {
        float trace = rotm[1, 1] + rotm[2, 2] + rotm[3, 3];
        float angle = (float)(Math.Acos((trace - 1) / 2));
        Vector3 axis = new Vector3();
        axis[0] = (float)((1 / (2 * Math.Sin(angle))) * (rotm[3, 2] - rotm[2, 3]));
        axis[1] = (float)((1 / (2 * Math.Sin(angle))) * (rotm[1, 3] - rotm[3, 1]));
        axis[2] = (float)((1 / (2 * Math.Sin(angle))) * (rotm[2, 1] - rotm[1, 2]));
        return angle * axis;
    }

    private Vector3 m2handle_pos(Vector3 tmp_posVec_HMD2handle, Matrix3x3 ground2HMD_rotm)
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

    private Vector3 wholeBodyGround2handle_pos(Vector3 tmp_posVec_HMD2handle, Matrix3x3 ground2HMD_rotm)
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
        // 基底座標轉至 Whole body 的大地座標
        double platform_yaw = Math.Atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y), 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z));
        Matrix3x3 wholeBodyGround2robotBase_rotm = new Matrix3x3((float)Math.Cos(platform_yaw), (float)-Math.Sin(platform_yaw), 0, (float)Math.Sin(platform_yaw), (float)Math.Cos(platform_yaw), 0, 0, 0, 1);
        Vector3 tmp_result_2 = Vector3.zero;
        tmp_result_2[0] = wholeBodyGround2robotBase_rotm[1, 1] * tmp_result[0] + wholeBodyGround2robotBase_rotm[1, 2] * tmp_result[1] + wholeBodyGround2robotBase_rotm[1, 3] * tmp_result[2];
        tmp_result_2[1] = wholeBodyGround2robotBase_rotm[2, 1] * tmp_result[0] + wholeBodyGround2robotBase_rotm[2, 2] * tmp_result[1] + wholeBodyGround2robotBase_rotm[2, 3] * tmp_result[2];
        tmp_result_2[2] = wholeBodyGround2robotBase_rotm[3, 1] * tmp_result[0] + wholeBodyGround2robotBase_rotm[3, 2] * tmp_result[1] + wholeBodyGround2robotBase_rotm[3, 3] * tmp_result[2];
        return tmp_result_2;
    }
}