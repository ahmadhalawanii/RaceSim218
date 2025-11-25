using System;
using System.Collections.Generic;
using UnityEngine;

public class checkpoints : MonoBehaviour
{
    [Header("Optional: Parent holding all checkpoints")]
    public Transform checkpointsParent;

    [Header("Ordered Checkpoints (in track sequence)")]
    public List<Transform> checkpointList;

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

        // Auto-fill from parent
        if (checkpointsParent != null && checkpointList.Count == 0)
        {
            checkpointList = new List<Transform>();
            foreach (Transform child in checkpointsParent)
            {
                checkpointList.Add(child);
            }

            // Sort numerically instead of alphabetically
            checkpointList.Sort((a, b) =>
            {
                int numA = ExtractNumber(a.name);
                int numB = ExtractNumber(b.name);
                return numA.CompareTo(numB);
            });
        }
    }

    // Extracts numeric part from a string (e.g., "checkpoint 12" -> 12)
    private int ExtractNumber(string s)
    {
        string digits = "";
        foreach (char c in s)
        {
            if (char.IsDigit(c))
                digits += c;
        }

        if (string.IsNullOrEmpty(digits))
            return 0;

        return int.Parse(digits);
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

    public void CarThroughCheckpoint(Transform checkpoint, Transform car)
    {
        if (!nextCheckpointIndexDict.ContainsKey(car))
            ResetCheckpoint(car);

        int expectedIndex = nextCheckpointIndexDict[car];

        if (checkpoint == checkpointList[expectedIndex])
        {
            nextCheckpointIndexDict[car] = (expectedIndex + 1) % checkpointList.Count;

            OnCarCorrectCheckpoint?.Invoke(this, new CarCheckpointEventArgs { carTransform = car });
        }
        else
        {
            OnCarWrongCheckpoint?.Invoke(this, new CarCheckpointEventArgs { carTransform = car });
        }
    }
}
