using System;
using System.Collections.Generic;
using UnityEngine;

public class checkpoints : MonoBehaviour
{
    [Header("Ordered Checkpoints (in track sequence)")]
    public List<Transform> checkpointList;

    // Per-car tracking: each car has its own next checkpoint index
    private Dictionary<Transform, int> nextCheckpointIndexDict;

    public class CarCheckpointEventArgs : EventArgs
    {
        public Transform carTransform;
    }

    public event EventHandler<CarCheckpointEventArgs> OnCarCorrectCheckpoint;
    public event EventHandler<CarCheckpointEventArgs> OnCarWrongCheckpoint;

    private void Awake()
    {
        nextCheckpointIndexDict = new Dictionary<Transform, int>();
    }

    public void ResetCheckpoint(Transform car)
    {
        if (nextCheckpointIndexDict.ContainsKey(car))
            nextCheckpointIndexDict[car] = 0;
        else
            nextCheckpointIndexDict.Add(car, 0);
    }

    public Transform GetNextCheckpoint(Transform car)
    {
        if (!nextCheckpointIndexDict.ContainsKey(car))
            ResetCheckpoint(car);

        int index = nextCheckpointIndexDict[car];
        return checkpointList[index];
    }

    // Called by each checkpoint trigger
    public void CarThroughCheckpoint(Transform checkpoint, Transform car)
    {
        if (!nextCheckpointIndexDict.ContainsKey(car))
            ResetCheckpoint(car);

        int expectedIndex = nextCheckpointIndexDict[car];

        if (checkpoint == checkpointList[expectedIndex])
        {
            // Correct checkpoint
            nextCheckpointIndexDict[car] = (expectedIndex + 1) % checkpointList.Count;

            OnCarCorrectCheckpoint?.Invoke(this, new CarCheckpointEventArgs
            {
                carTransform = car
            });
        }
        else
        {
            // Wrong checkpoint
            OnCarWrongCheckpoint?.Invoke(this, new CarCheckpointEventArgs
            {
                carTransform = car
            });
        }
    }
}
