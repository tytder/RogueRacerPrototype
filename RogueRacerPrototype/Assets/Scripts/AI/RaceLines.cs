using System;
using System.Linq;
using Sirenix.OdinInspector;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;
using UnityUtils;

public class RaceLines : MonoBehaviour
{
    [Serializable]
    private enum WaypointAmountType
    {
        Total,
        PerSplineUnit,
        PerKnot
    }

    [SerializeField] private float _carHalfWidth;
    [SerializeField] private float _amountOfWaypoints;
    [SerializeField] private float _raceLineResolution;
    [SerializeField] private WaypointAmountType _waypointAmountType;
    //[SerializeField] private WaypointDebugDisplayType _waypointDebugDisplayType;
    [SerializeField] private bool 
        _debugPositionDrawWaypoint, 
        _debugTangentDrawWaypoint, 
        _debugAccelDrawWaypoint, 
        _debugAccelDeltaFrontDrawWaypoint, 
        _debugAccelDeltaRearDrawWaypoint, 
        _debugIsDrawingTrack;
    [SerializeField] private SelfGeneratedRoadMesh _roadData;
    [SerializeField] private int _amountOfIterations = 20;
    [SerializeField] private SplineContainer _racingLineVisualizer;

    private Vector3[] _positionsOnPath;
    private Vector3[] _tangents;
    private Vector3[] _racingLine;
    
    [HideInInspector]
    public Waypoint[] Waypoints;
    
    private float[] _displacements;
    private Spline _path;
    private float _trackWidth;
    
    [ReadOnly] [SerializeField] 
    private int _waypointAmount;


    [Button("Generate Racing Line")]
    private void Awake()
    {
        if (!enabled) return;

        _path = _roadData.SplineContainer.Spline;
        _trackWidth = _roadData.RoadWidth - _carHalfWidth;

        var raceLineControlPointsAmount = Mathf.CeilToInt(_path.Count * _raceLineResolution);

        //_racingLine = _path.Select(p => (Vector3)p.Position).ToArray();
        _tangents = new Vector3[raceLineControlPointsAmount];
        _positionsOnPath = new Vector3[raceLineControlPointsAmount];
        
        // filling the positionsOnPath and _tangents arrays
        switch (_waypointAmountType)    
        {
            case WaypointAmountType.PerKnot:
                int curveCount = _path.GetCurveCount();
                float knotsPerCurve = _raceLineResolution;
                for (int curveIndex = 0; curveIndex < curveCount; curveIndex++)
                {
                    var currentCurve = _path.GetCurve(curveIndex);
                    for (int i = 0; i < knotsPerCurve; i++)
                    {
                        float localT = i / knotsPerCurve;
                        int waypointCount = curveIndex * (int)knotsPerCurve + i;
                        _positionsOnPath[waypointCount] = CurveUtility.EvaluatePosition(currentCurve, localT);
                        _tangents[waypointCount] = CurveUtility.EvaluateTangent(currentCurve, localT);
                    }
                }
                break;
            default:
                throw new NotImplementedException();
        }

        // set pre iterations positions
        _racingLine = _positionsOnPath.ToArray();
        _displacements = new float[_racingLine.Length];

        for (int iteration = 0; iteration < _amountOfIterations; iteration++)
        {
            // Getting the _displacement values
            for (int j = 0; j < _racingLine.Length; j++)
            {
                Vector3 left = _racingLine[(j - 1 + _racingLine.Length) % _racingLine.Length];
                Vector3 right = _racingLine[(j + 1) % _racingLine.Length];
                Vector3 middle = _racingLine[j];

                // Vectors to neighbors
                Vector3 vL = (left - middle).normalized;
                Vector3 vR = (right - middle).normalized;

                // Bisector
                Vector3 bisector = vL + vR;
                bisector = bisector.sqrMagnitude > 1e-4f ? 
                    bisector.With(y: 0).normalized : 
                    Vector3.zero;

                // Tangent at this sample
                Vector3 tangent = _tangents[j].With(y: 0).normalized;

                // Project onto 2D plane (XZ)
                //tangent.y = 0;
                //bisector.y = 0;

                // perp dot
                float dp = -tangent.z * bisector.x + tangent.x * bisector.z;

                // shortest distance around track
                _displacements[j] += dp * 0.3f;
                //_displacements[j] = Mathf.Lerp(_displacements[j], _displacements[j] + dp * 0.3f, 0.8f);
                
                _displacements[(j + 1) % _racingLine.Length] += dp * -0.1f;
                _displacements[(j - 1 + _racingLine.Length) % _racingLine.Length] += dp * -0.1f;
            }

            // Clamp displaced points to track width
            for (int j = 0; j < _racingLine.Length; j++)
            {
                if (_displacements[j] >= _trackWidth)
                    _displacements[j] = _trackWidth;
                if (_displacements[j] <= -_trackWidth)
                    _displacements[j] = -_trackWidth;

                //float t = j / ((float)_racingLine.Length - 1);
                var g = _tangents[j];
                // float glen = Mathf.Sqrt(g.x * g.x + g.z * g.z);
                // g.x /= glen;
                // g.z /= glen;
                var gn = g.With(y: 0).normalized;

                //float3 positionOnPath = _path.EvaluatePosition(t);
                float3 positionOnPath = _positionsOnPath[j];

                _racingLine[j] = new Vector3(
                    positionOnPath.x + -gn.z * _displacements[j],
                    positionOnPath.y,
                    positionOnPath.z + gn.x * _displacements[j]
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

        GenerateWaypoints();
    }

    [Button("Generate Waypoints")]
    private void GenerateWaypoints()
    {
        _waypointAmount = _waypointAmountType switch
        {
            WaypointAmountType.Total => (int)_amountOfWaypoints,
            WaypointAmountType.PerSplineUnit => Mathf.CeilToInt(_roadData.SplineContainer.CalculateLength(0) *
                                                                _amountOfWaypoints),
            WaypointAmountType.PerKnot => Mathf.CeilToInt(_path.Count * _raceLineResolution * _amountOfWaypoints),
            _ => throw new ArgumentOutOfRangeException()
        };
        Waypoints = new Waypoint[_waypointAmount];
        switch (_waypointAmountType)    
        {
            case WaypointAmountType.PerKnot:
                for (int i = 0; i < transform.childCount; i++)
                {
                    DestroyImmediate(transform.GetChild(i).gameObject);
                }
                
                int curveCount = _racingLineVisualizer.Spline.GetCurveCount();
                float knotsPerCurve = _amountOfWaypoints;
                for (int curveIndex = 0; curveIndex < curveCount; curveIndex++)
                {
                    var currentCurve = _racingLineVisualizer.Spline.GetCurve(curveIndex);
                    for (int i = 0; i < knotsPerCurve; i++)
                    {
                        float localT = i / knotsPerCurve;
                        int waypointCount = curveIndex * (int)knotsPerCurve + i;
                        var prevWaypointIndex = (waypointCount - 1 + Waypoints.Length) % Waypoints.Length;
                        var waypointObj = new GameObject("waypoint "+ waypointCount);
                        waypointObj.transform.SetParent(transform);
                        var waypoint = waypointObj.AddComponent<Waypoint>();
                        waypoint.Position = (Vector3)CurveUtility.EvaluatePosition(currentCurve, localT) + transform.position;
                        waypoint.AccelVector = CurveUtility.EvaluateAcceleration(currentCurve, localT);
                        waypoint.TangentVector = CurveUtility.EvaluateTangent(currentCurve, localT);
                        waypointObj.transform.forward = waypoint.TangentVector.With(y:0).normalized;
                        Waypoints[waypointCount] = waypoint;
                        
                        var triggerCollider = waypointObj.AddComponent<BoxCollider>();
                        triggerCollider.size = new Vector3(_trackWidth*5, 10f, 10f);
                        triggerCollider.isTrigger = true;
                        
                        if (Waypoints[prevWaypointIndex])
                            Waypoints[waypointCount].PreviousWaypoint = Waypoints[prevWaypointIndex];
                        if (waypointCount != 0 && Waypoints[waypointCount - 1])
                            Waypoints[waypointCount - 1].NextWaypoint = Waypoints[waypointCount];
                        
                        if (waypointCount >= _waypointAmount - 1)
                        {
                            Waypoints[0].PreviousWaypoint = Waypoints[waypointCount];
                        }
                    }
                }
                break;
            default:
                throw new NotImplementedException();
                /*for (int i = 0; i < _waypointAmount; i++)
                {
                    float t = i / ((float)_waypointAmount - 1);
                    //var waypoint = new GameObject("waypoint" + i).AddComponent<Waypoint>();

                    //_path.SplineToCurveT(t, out var curveT);
                    //waypoint.transform.position = (Vector3)_path.EvaluatePosition(curveT) + transform.position;
                    
                    var prevWaypoint = Waypoints[(i - 1 + Waypoints.Length) % Waypoints.Length];
                    var nextWaypoint = Waypoints[(i + 1) % Waypoints.Length];
                    Waypoints[i] = new Waypoint
                        {
                            Position = _racingLineVisualizer.EvaluatePosition(t),
                            AccelVector = _racingLineVisualizer.EvaluateAcceleration(t),
                            TangentVector = _racingLineVisualizer.EvaluateTangent(t),
                            PreviousWaypoint = prevWaypoint,
                            NextWaypoint = nextWaypoint
                        };
                }
                break;*/
        }
    }

    private void OnDrawGizmos()
    {
        if (Waypoints == null) return;
        if (_debugIsDrawingTrack)
        {
            for (int i = 0; i < _tangents.Length; i++)
            {
                var pos = _positionsOnPath[i] + transform.position;
                var tan = _tangents[i];
                DrawWaypointPosition(pos);
                DrawWaypointTangentVector(pos, tan);
            }
        }
        else
        {
            foreach (var waypoint in Waypoints)
            {
                if (_debugPositionDrawWaypoint)
                    DrawWaypointPosition(waypoint.Position);
                if (_debugTangentDrawWaypoint)
                    DrawWaypointTangentVector(waypoint);
                if (_debugAccelDrawWaypoint)
                    DrawWaypointAccelVector(waypoint);
                if (_debugAccelDeltaFrontDrawWaypoint)
                    DrawWaypointAccelDeltaFront(waypoint);
                if (_debugAccelDeltaRearDrawWaypoint)
                    DrawWaypointAccelDeltaRear(waypoint);
            }
        }
    }

    private void DrawWaypointAccelDeltaFront(Waypoint waypoint)
    {
        Gizmos.color = waypoint.NextWaypointAccelDelta > 0 ? Color.green : Color.red;
        Gizmos.DrawLine(
            waypoint.Position + waypoint.TangentVector.normalized,
            waypoint.Position + waypoint.TangentVector.normalized + Vector3.up * Mathf.Abs(waypoint.NextWaypointAccelDelta)
            );
    }
    private void DrawWaypointAccelDeltaRear(Waypoint waypoint)
    {
        float delta = waypoint.PrevWaypointAccelDelta;
        //float delta = (waypoint.PrevWaypointAccelDelta + waypoint.NextWaypointAccelDelta) / 2f;
        Gizmos.color = delta > 0 ? 
            (_debugAccelDeltaFrontDrawWaypoint ? Color.dodgerBlue : Color.green) : 
            (_debugAccelDeltaFrontDrawWaypoint ? Color.rebeccaPurple : Color.red);
        Gizmos.DrawLine(
            waypoint.Position - waypoint.TangentVector.normalized,
            waypoint.Position - waypoint.TangentVector.normalized + Vector3.up * Mathf.Abs(delta)
            );
    }

    private void DrawWaypointTangentVector(Waypoint waypoint)
    {
        DrawWaypointTangentVector(waypoint.Position, waypoint.TangentVector);
        Gizmos.color = Color.white;
        Gizmos.DrawLine(waypoint.Position, waypoint.Position + Vector3.up * waypoint.TangentVector.With(y:0).magnitude);
    }
    
    private void DrawWaypointTangentVector(Vector3 position, Vector3 tangent)
    {
        Gizmos.color = Color.orange;
        Gizmos.DrawLine(position, position + tangent);
    }

    private void DrawWaypointAccelVector(Waypoint waypoint)
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(waypoint.Position, waypoint.Position + waypoint.AccelVector.With(y:0));
        Gizmos.color = Color.white;
        Gizmos.DrawLine(waypoint.Position, waypoint.Position + Vector3.up * waypoint.AccelVector.magnitude);
    }

    private void DrawWaypointPosition(Vector3 position)
    {
        Gizmos.color = Color.dimGray;
        Gizmos.DrawSphere(position, 1f);
    }

    /*

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
    }*/
}