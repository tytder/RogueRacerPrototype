using System;
using UnityEngine;
using UnityUtils;

public class AICarInputGpt : MonoBehaviour, ICarInput
{
    [Header("Waypoints")]
    public float lookaheadDistance = 10f; // for obstacle

    [Header("Steering")]
    public float maxSteeringAngle = 30f;

    [Header("Speed Settings")]
    [Range(0,1)]public float minCornerSpeedPercentage = .25f;
    public float curvatureScale = 10f;
    public float cornerBrakeBias = 0.7f;
    public float cornerReleaseBias = 1.2f;

    public float Move => _move;
    public float Steer => _steer;
    public bool SetCurrentWaypoint(Waypoint currentWaypoint)
    {
        bool ret = _currentWaypoint;
        _currentWaypoint = currentWaypoint;
        return ret;
    }

    private float _move;  // throttle [-1 reverse .. 1 forward]
    private float _steer; // steering [-1 left .. 1 right]

    private float _prevCurvature = 0f;

    private Waypoint _currentWaypoint;
    private CarController _carController;

    private void Awake()
    {
        _carController = GetComponent<CarController>();
    }

    void Update()
    {
        if (!_currentWaypoint) return;
        Vector3 direction = (_currentWaypoint.NextWaypoint.Position - transform.position);

        direction.y = 0;
        Vector3 dirNorm = direction.normalized;

        // Steering
        float angle = Vector3.SignedAngle(transform.forward, dirNorm, Vector3.up);
        _steer = Mathf.Clamp(angle / maxSteeringAngle, -1f, 1f);

        // Calculate curvature-based target speed
        float curvature = _currentWaypoint.AccelVector.With(y:0).magnitude;
        float maxSpeed = _carController._maxSpeedForward;
        float baseTarget = Mathf.Lerp(maxSpeed * minCornerSpeedPercentage, maxSpeed,
                                      1f / (1f + curvature * curvatureScale));

        // Curvature delta braking or throttle
        float delta = curvature - _prevCurvature;
        if (delta > 0) baseTarget *= cornerBrakeBias;
        else if (delta < 0) baseTarget *= cornerReleaseBias;

        _prevCurvature = curvature;

        // Obstacle detection (raycast ahead)
        if (Physics.Raycast(transform.position, transform.forward, lookaheadDistance))
        {
            _move = Mathf.Lerp(_move, -1f, Time.deltaTime * 5f);
        }
        else
        {
            float speed = _carController.CurrentCarLocalVelocity.z;
            _move = speed > baseTarget ? -1f : 1f;
        }
    }
}

