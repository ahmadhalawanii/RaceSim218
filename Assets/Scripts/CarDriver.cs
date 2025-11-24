using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarDriver : MonoBehaviour
{
    public float maxMotorTorque = 1500f;   // Acceleration force
    public float maxSteeringAngle = 30f;   // Steering angle
    public float brakeForce = 3000f;

    public WheelCollider frontLeftCollider;
    public WheelCollider frontRightCollider;
    public WheelCollider rearLeftCollider;
    public WheelCollider rearRightCollider;

    public Transform frontLeftMesh;
    public Transform frontRightMesh;
    public Transform rearLeftMesh;
    public Transform rearRightMesh;

    private float motorInput = 0f;
    private float steeringInput = 0f;

    private bool controlledByML = false;

    // ********** REQUIRED FOR AGENT **********
    public void SetInputs(float forward, float turn)
    {
        motorInput = forward;
        steeringInput = turn;
        controlledByML = true;
    }

    // ********** REQUIRED FOR AGENT **********
    public void StopCompletely()
    {
        motorInput = 0f;
        steeringInput = 0f;

        frontLeftCollider.brakeTorque = brakeForce;
        frontRightCollider.brakeTorque = brakeForce;
        rearLeftCollider.brakeTorque = brakeForce;
        rearRightCollider.brakeTorque = brakeForce;
    }
    // ****************************************

    void Update()
    {
        // Only use keyboard input in Heuristic (player controlled)
        if (!controlledByML)
        {
            motorInput = Input.GetAxis("Vertical");
            steeringInput = Input.GetAxis("Horizontal");
        }

        UpdateWheelVisual(frontLeftCollider, frontLeftMesh);
        UpdateWheelVisual(frontRightCollider, frontRightMesh);
        UpdateWheelVisual(rearLeftCollider, rearLeftMesh);
        UpdateWheelVisual(rearRightCollider, rearRightMesh);
    }

    void FixedUpdate()
    {
        // Steering
        frontLeftCollider.steerAngle = steeringInput * maxSteeringAngle;
        frontRightCollider.steerAngle = steeringInput * maxSteeringAngle;

        // Motor torque
        rearLeftCollider.motorTorque = motorInput * maxMotorTorque;
        rearRightCollider.motorTorque = motorInput * maxMotorTorque;

        // Brake when no input
        if (motorInput == 0f)
        {
            frontLeftCollider.brakeTorque = brakeForce;
            frontRightCollider.brakeTorque = brakeForce;
            rearLeftCollider.brakeTorque = brakeForce;
            rearRightCollider.brakeTorque = brakeForce;
        }
        else
        {
            frontLeftCollider.brakeTorque = 0f;
            frontRightCollider.brakeTorque = 0f;
            rearLeftCollider.brakeTorque = 0f;
            rearRightCollider.brakeTorque = 0f;
        }

        // Reset ML flag each frame so Update knows if agent sent inputs
        controlledByML = false;
    }

    void UpdateWheelVisual(WheelCollider col, Transform wheelMesh)
    {
        Vector3 pos;
        Quaternion rot;
        col.GetWorldPose(out pos, out rot);
        wheelMesh.position = pos;
        wheelMesh.rotation = rot;
    }
}
