using System;
using UnityEngine;
using UnityUtils;

public class AICarInput : MonoBehaviour, ICarInput
{
    [SerializeField] private float _maxSteeringAngle = 30f;
    [SerializeField] private float _distanceThreshold;

    public float Move => MoveLogic();

    private float MoveLogic()
    {
        /*if (!_currentWaypoint) return 0f;
        if (Vector3.Dot(transform.forward, DirToMove) <= 0) return -1;
        return _currentWaypoint.NextWaypoint.TangentVector.With(y: 0).magnitude /
               (_maxTangentSize / 1.5f);*/
        if (Vector3.Dot(transform.forward, DirToMove) <= 0) return -1;
        if (Physics.Raycast(transform.position, transform.forward, out RaycastHit hit, _distanceThreshold))
        {
            Debug.DrawLine(transform.position, hit.point, Color.red);
            if (hit.distance < 2)
            {
                return -1f;
            }
            else
            {
                return (hit.distance) / (_distanceThreshold);
            }
        }

        return 1f;
    }

    public float Steer => SteerLogic();

    private float SteerLogic()
    {
        if (!_currentWaypoint) return 0f;
        
        /*float currentSteer = Mathf.Clamp(
            Vector3.SignedAngle(
                transform.forward.With(y: 0).normalized,
                DirToMove,
                Vector3.up) 
            / _maxSteeringAngle, -1f,1f);

        float returnAmount = Mathf.Lerp(_previousSteerAmount, currentSteer, .5f);
        
        _previousSteerAmount = currentSteer;

        return returnAmount;*/
        return Mathf.Clamp(
            Vector3.SignedAngle(
                transform.forward.With(y: 0).normalized,
                _currentWaypoint.TangentVector.With(y:0).normalized,
                Vector3.up) 
            / _maxSteeringAngle, -1f,1f);
    }
    
    private Vector3 DirToMove => (_currentWaypoint.NextWaypoint.Position - transform.position).With(y:0).normalized;
    private float _previousSteerAmount;
    
    private float _maxTangentSize;
    
    private Waypoint _currentWaypoint;
    
    private void Start()
    {
        _maxTangentSize = 0;
        var waypoints = FindFirstObjectByType<RaceLines>().Waypoints;
        foreach (var waypoint in waypoints)
        {
            float tangentSize = waypoint.TangentVector.With(y: 0).magnitude;
            if (tangentSize > _maxTangentSize)
                _maxTangentSize = tangentSize;
        }

        //_currentWaypoint = waypoints[0];
    }
    
    public bool SetCurrentWaypoint(Waypoint currentWaypoint)
    {
        bool ret = _currentWaypoint;
        _currentWaypoint = currentWaypoint;
        return ret;
    }

    private void OnDrawGizmos()
    {
        if (!_currentWaypoint) return;
        Gizmos.color = Color.black;
        Gizmos.DrawSphere(_currentWaypoint.Position + Vector3.up * 10, 1);
        Gizmos.DrawSphere(_currentWaypoint.Position + Vector3.up * 9, 1);
        Gizmos.DrawSphere(_currentWaypoint.Position + Vector3.up * 8, 1);
        Gizmos.DrawSphere(_currentWaypoint.Position + Vector3.up * 7, 1);
        Gizmos.DrawSphere(_currentWaypoint.Position + Vector3.up * 6, 1);
        Gizmos.DrawSphere(_currentWaypoint.Position + Vector3.up * 5, 1);
    }
}