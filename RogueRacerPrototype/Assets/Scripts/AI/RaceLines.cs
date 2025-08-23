using System;
using System.Collections.Generic;
using System.Linq;
using Sirenix.OdinInspector;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

public class RaceLines : MonoBehaviour
{
    [Serializable] private enum WaypointAmountType
    {
        Total,
        PerSplineUnit
    }
    [Serializable] private enum DebugDisplay
    {
        Waypoints,
        WaypointsPersistent,
        SplineTangent,
        SplineTangentPersistent,
        Both,
        BothPersistent
    }
    
    [SerializeField] private int _amountOfWaypoints;
    [SerializeField] private WaypointAmountType _waypointAmountType;
    [SerializeField] private DebugDisplay _debugDisplay;
    [SerializeField] private SelfGeneratedRoadMesh _roadData;
    [SerializeField] private int _amountOfIterations = 20;
    [SerializeField] private SplineContainer _racingLineVisualizer;

    private Vector3[] _racingLine;
    private float[] _displacements;
    private Spline _path;
    private float _trackWidth;

    [Button("GenerateRaceLine")]
    private void Awake()
    {
        if (!enabled) return;
        
        _path = _roadData.SplineContainer.Spline;
        _trackWidth = _roadData.RoadWidth;
        
        
        int waypointAmount = _waypointAmountType switch
        {
            WaypointAmountType.Total => _amountOfWaypoints,
            WaypointAmountType.PerSplineUnit => Mathf.CeilToInt(_roadData.SplineContainer.CalculateLength(0)) *  _amountOfWaypoints,
            _ => throw new ArgumentOutOfRangeException()
        };
        _racingLine = new Vector3[waypointAmount];
        for (int i = 0; i < waypointAmount; i++)
        {
            float t = i / ((float)waypointAmount - 1);
            // todo: change back to knots and find way to convert knot position to t value to get inbetween knot positions
            _racingLine[i] = _path.EvaluatePosition(t);
        }
        
        _displacements = new float[waypointAmount];
        
        for (int i = 0; i < _amountOfIterations; i++)
        {
            for (int j = 0; j < _racingLine.Length; j++)
            {
                Vector3 left = _racingLine[(i - 1 + _racingLine.Length) % _racingLine.Length];
                Vector3 right = _racingLine[(i + 1) % _racingLine.Length];
                Vector3 middle = _racingLine[i];

                // Vectors to neighbors
                Vector3 vL = (left - middle).normalized;
                Vector3 vR = (right - middle).normalized;

                // Bisector
                Vector3 bisector = (vL + vR).normalized;

                // Tangent at this sample
                float t = j / (float)(_racingLine.Length - 1);
                Vector3 tangent = ((Vector3)_path.EvaluateTangent(t)).normalized;

                // Project onto 2D plane (XZ)
                tangent.y = 0;
                bisector.y = 0;

                // perp dot
                float dp = -tangent.z * bisector.x + tangent.x * bisector.z;

                _displacements[i] += dp * 0.3f;
            }

            // Clamp displaced points to track width
            for (int j = 0; j < _racingLine.Length; j++)
            {
                if (_displacements[j] >= _trackWidth) 
                    _displacements[j] = _trackWidth;
                if (_displacements[j] <= -_trackWidth) 
                    _displacements[j] = -_trackWidth;
                
                float t = j / ((float)_racingLine.Length - 1);
                var g = _path.EvaluateTangent(t);
                float glen = Mathf.Sqrt(g.x * g.x + g.z * g.z);
                g.x /= glen;
                g.z /= glen;

                float3 positionOnPath = _path.EvaluatePosition(t);
                
                _racingLine[j] = new Vector3(
                    positionOnPath.x + -g.y * _displacements[j],
                    positionOnPath.y,
                    positionOnPath.z + g.x * _displacements[j]
                );
            }
        }
        
        // === Convert to Unity Spline ===
        if (_racingLineVisualizer == null)
        {
            _racingLineVisualizer = gameObject.AddComponent<SplineContainer>();
        }

        _racingLineVisualizer.Spline = new Spline();

        for (int i = 0; i < _racingLine.Length; i++)
        {
            BezierKnot knot = new BezierKnot(_racingLine[i]);
            _racingLineVisualizer.Spline.Add(knot);
        }

        _racingLineVisualizer.Spline.Closed = true;
        _racingLineVisualizer.Spline.SetTangentMode(TangentMode.AutoSmooth);
    }
    
    private void OnDrawGizmos()
    {
        if (_racingLine == null) return;
        if ((int)_debugDisplay % 2 != 1) return;

        switch (_debugDisplay)
        {
            case DebugDisplay.WaypointsPersistent:
                foreach (Vector3 waypoint in _racingLine)
                {
                    Gizmos.DrawSphere(waypoint, 1f);
                }
                break;
            case DebugDisplay.SplineTangentPersistent:
                break;
            case DebugDisplay.BothPersistent:
                break;
        }
    }
    
    private void OnDrawGizmosSelected()
    {
        if (_racingLine == null) return;
        if ((int)_debugDisplay % 2 != 0) return;
        
        switch (_debugDisplay)
        {
            case DebugDisplay.Waypoints:
                foreach (Vector3 waypoint in _racingLine)
                {
                    Gizmos.DrawSphere(waypoint, 1f);
                }
                break;
            case DebugDisplay.SplineTangent:
                break;
            case DebugDisplay.Both:
                break;
        }
    }
}
