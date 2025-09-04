using System;
using System.Collections;
using System.Linq;
using PrimeTween;
using Sirenix.OdinInspector;
using Sirenix.Serialization;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;
using UnityUtils;

public class CarController : MonoBehaviour
{
    [FormerlySerializedAs("_playerInput")]
    [Header("References")] 
    public InputReader PlayerInput;
    [FormerlySerializedAs("_carRb")] public Rigidbody CarRb;
    [SerializeField] private Transform[] _suspensionRayPoints;
    [SerializeField] private LayerMask _drivable;
    [SerializeField] private Transform _accelerationPoint;
    [SerializeField] private GameObject[] _tires = new  GameObject[4];
    [SerializeField] private GameObject[] _frontTireParents = new  GameObject[2];
    
    [HideInInspector] public ICarInput CarInput;
    
    [Header("Suspension Settings")]
    [Tooltip("Max force spring can exert when fully compressed.")]
    [SerializeField] private float _springStiffness;
    [Tooltip("Rest length is when spring is not compressed nor stretched.")]
    [SerializeField] private float _restLengthFront;
    [Tooltip("Rest length is when spring is not compressed nor stretched.")]
    [SerializeField] private float _restLengthBack;
    [Tooltip("Max distance spring can compress/stretch from its restLength.")]
    [SerializeField] private float _springMaxTravel;
    [SerializeField] private float _wheelRadiusFront;
    [SerializeField] private float _wheelRadiusBack;
    [Range(.2f,1f)] [Tooltip("The lower the value, the bouncier the spring.")]
    [SerializeField] private float _damperZeta;
    public bool IsGrounded => _wheelsIsGrounded.Any();
    private bool[] _wheelsIsGrounded =  new bool[4];
    
    [ReadOnly][ShowInInspector]
    private float _damperStiffness;

    [Header("Steering")] 
    [Tooltip("x-axis: % of tire velocity in perpendicular direction,\n y-axis: % of traction")] 
    [SerializeField] public AnimationCurve _frontTiresGrip;
    [Tooltip("x-axis: % of tire velocity in perpendicular direction,\n y-axis: % of traction")] 
    [SerializeField] public AnimationCurve _rearTiresGrip;

    [SerializeField] private float _steerStrength = 15f;
    [SerializeField] private float _steerStrength2 = 15f;
    [SerializeField] private AnimationCurve _turningCurve;
    [SerializeField] private float _dragCoefficient = 1f;
    [SerializeField] private float _wheelCircumference;
    public float HandlingMultiplier = 1f;

    private float _steerInput;
    
    [Header("Acceleration")]
    [Tooltip("x-axis: car's speed as % of top speed,\n y-axis: % of available torque")]
    [SerializeField] public AnimationCurve _engineTorque;
    public float _maxSpeedForward = 100f;
    [SerializeField] private float _maxSpeedBackwards = 30f;
    [SerializeField] private float _acceleration = 25f;
    [SerializeField] private float _acceleration2 = 5f;
    [SerializeField] private float _deceleration = 10f;
    [SerializeField] private float _brakingDeceleration = 100f;
    [SerializeField] private float _brakingDragCoefficient = .5f;

    [HideInInspector] public Vector3 CurrentCarLocalVelocity;
    private float _carVelocityRatio;
    private float _moveInput;
    
    [Header("Engine Settings")] 
    public float _idleRpm = 900f;
    public float _maxRpm = 9000f;
    [SerializeField] private float _maxUsableRpm = 8100f;
    [SerializeField] private float _throttleRpmIncreaseRate = 3000f; // rpm per second
    [SerializeField] private float _throttleRpmDecreaseRate = 2500f; // rpm per second
    [SerializeField] private float _upshiftRpm = 7000f; 
    [SerializeField] private float _downshiftRpm = 3500f; 
    [SerializeField] private float[] _gearRatios = { 3.5f, 2.2f, 1.6f, 1.2f, 1.0f, 0.85f };
    [SerializeField] private float[] _gearTorqueRatios = { .7f, .85f, 1f, 1f, 1f, 1f };
    [SerializeField] private float _finalDriveRatio = 3.9f;
    [SerializeField] private float _engineTune = .5f;
    [SerializeField] public AnimationCurve _torqueCurve;
    [SerializeField] private bool _isManual;
    [SerializeField] private TMP_Text _currentGearDisplay;

    private int _currentGear;
    public float EngineRpm;
    private float _wheelRpm;
    private float _previousEngineRpm;
    
    [Header("Visuals")]
    [SerializeField] private float _tireRotationSpeed = 3000f;
    [SerializeField] private float _maxSteeringAngle = 30f;
    [SerializeField] private Image _steerLeft;
    [SerializeField] private Image _steerRight;
    [SerializeField] private Image _braking;
    [SerializeField] private GameObject _isBrakingDisplay;
    [SerializeField] private Image _gas;
    [SerializeField] private TMP_Text _speedometerText;
    [SerializeField] private Vector2 _minMaxSpeedArrowAngle;
    [SerializeField] private RectTransform _speedometerArrow;
    [SerializeField] private TMP_Text _racePosition;
    [SerializeField] private TMP_Text _lapCounter;
    
    private bool _isBraking;

    public float[] DEBUG_GripCurve = new float[4];
    public float DEBUG_EngineTorqueCurve;
    private bool _rpmIsAnimating;

    private int _lapCount;
    private int _waypointCount;
    private float _totalWaypointCount;
    private Waypoint _nextWaypoint;
    private CarController[] _allCars;

    public int WaypointCount
    {
        get => _waypointCount;
        set
        {
            _waypointCount = value;
            LapCount = (int)(_waypointCount / _totalWaypointCount);
        }
    }

    public int RacePosition;
    public float AbsolutePos => WaypointCount * 1000000 + Vector3.Distance(transform.position, _nextWaypoint.Position);

    private int FindPosition()
    {
        /*Array.Sort(_allCars, (a, b) => b.AbsolutePos.CompareTo(a.AbsolutePos));

        int playerIndex = Array.FindIndex(_allCars, car => car.PlayerInput);
        return playerIndex + 1;*/
        int position = 1; // start at 1st
        foreach (var car in _allCars)
        {
            if (car && !car.PlayerInput)
            {
                if (car.AbsolutePos > AbsolutePos)
                    position++;
            }
        }
        return position;
    }
    
    private IEnumerator UpdatePositionsRoutine()
    {
        while (true)
        {
            try
            {
                RacePosition = FindPosition();
            }
            catch (Exception e)
            {
                Debug.Log(e.Message);
            }
            yield return new WaitForSeconds(.1f);
            if (!RaceManager.Instance.IsRaceActive && _lapCount > 0)
                break;
        }
    }  

    public int LapCount
    {
        get => _lapCount;
        set
        {
            _lapCount = value;
            if (!PlayerInput) return;
            _lapCounter.text = $"Lap {_lapCount + 1} / {RaceManager.Instance.TotalAmountOfLaps}";
            if (_lapCount == RaceManager.Instance.TotalAmountOfLaps)
            {
                RaceManager.Instance.RaceCompleted();
            }
        }
    }

    private void Start()
    {
        if (!TryGetComponent(out CarInput))
        {
            CarInput = PlayerInput;
        }
        _damperStiffness = 2 * _damperZeta * Mathf.Sqrt(_springStiffness * CarRb.mass);
        EngineRpm = _idleRpm;

        _totalWaypointCount = FindFirstObjectByType<RaceLines>().Waypoints.Length;
        _allCars = FindObjectsByType<CarController>(FindObjectsSortMode.None);
        
        if (PlayerInput)
        {
            for (int i = 0; i < _engineTorque.keys.Length; i++)
            {
                if (i < _engineTorque.keys.Length * .4f)
                {
                    _engineTorque.keys[i].value *= GameSingleton.Instance.PlayerAccelMultiplier;
                }
                else
                {
                    _engineTorque.keys[i].value *= GameSingleton.Instance.PlayerSpeedMultiplier;
                }
            }

            HandlingMultiplier = GameSingleton.Instance.PlayerHandlingMultiplier;
            
            _lapCounter.text = $"Lap {_lapCount + 1} / {RaceManager.Instance.TotalAmountOfLaps}";
            RacePosition = _allCars.Length;
        }
        //_carRb
        //Application.targetFrameRate = 30; // TODO: power saving.
    }

    public bool SetCurrentWaypoint(Waypoint waypoint)
    {
        WaypointCount++;
        if (PlayerInput)
        {
            bool start = !_nextWaypoint;
            if (start && !IsInvoking(nameof(UpdatePositionsRoutine)))  
                StartCoroutine(nameof(UpdatePositionsRoutine));
        }
        _nextWaypoint = waypoint.NextWaypoint;
        return CarInput.SetCurrentWaypoint(waypoint);
    }

    private void Update()
    {
        if (PlayerInput)
        {
            UiVisuals();
        }
        if (!RaceManager.Instance.IsRaceActive)
        {
            return;
        }
        _moveInput = CarInput.Move;
        _steerInput = CarInput.Steer;
    }

    private void FixedUpdate()
    {
        if (!RaceManager.Instance.IsRaceActive)
        {
            CarRb.linearVelocity = Vector3.zero;
            for (int i = 0; i < _suspensionRayPoints.Length; i++)
            {
                Transform rayPoint = _suspensionRayPoints[i];
                float maxLength = _restLengthFront + _springMaxTravel; // (i < 2 ? _restLengthFront : _restLengthBack)
                if (Physics.Raycast(rayPoint.position, -rayPoint.up, out var hit, 10f, _drivable))
                {
                    HandleSuspension(hit, rayPoint);
                    if (hit.distance < maxLength + _wheelRadiusFront)
                    {
                        _wheelsIsGrounded[i] = true;
                        SetTirePosition(_tires[i], hit.point + rayPoint.up * _wheelRadiusFront);
                    }
                    else
                    {
                        _wheelsIsGrounded[i] = false;
                        SetTirePosition(_tires[i], rayPoint.position - rayPoint.up * (maxLength - _wheelRadiusFront));
                    }
                }
                else
                {
                    _wheelsIsGrounded[i] = false;
                
                    SetTirePosition(_tires[i], rayPoint.position - rayPoint.up * (maxLength - _wheelRadiusFront));
                }
            }
            return;
        }
        CalculateCarVelocity();
        UpdateEngineRpm();
        if (!_isManual)
        {
            AutoStartAndReverse();
            CheckAutoShift();
        }
        WheelFunctions();
        HandleDeceleration();
        SidewaysDrag();
        Visuals();
        _previousEngineRpm = EngineRpm;
    }
    
    private void CalculateCarVelocity()
    {
        CurrentCarLocalVelocity = transform.InverseTransformDirection(CarRb.linearVelocity);
        _carVelocityRatio = CurrentCarLocalVelocity.z / _maxSpeedForward;

        _isBraking = Mathf.Abs(_moveInput) > 0.05f && Math.Sign(CurrentCarLocalVelocity.z) != Math.Sign(_moveInput);
        _isBrakingDisplay?.SetActive(_isBraking);
    }

    
    #region GeneralCarForces
    
    /*private void Movement()
    {
        HandleDeceleration();
        if (IsGrounded)
        {
            HandleAcceleration();
            Turn();
            SidewaysDrag();
        }
    }*/

    /*
    private void HandleAcceleration()
    {
        float availableTorque = _engineTorque.Evaluate(_carVelocityRatio) * _moveInput * _acceleration;
        if (_currentCarLocalVelocity.z < _maxSpeed)
        {
            _carRb.AddForceAtPosition(availableTorque * transform.forward.With(y: 0), _accelerationPoint.position,
                ForceMode.Acceleration);
        }
    }*/

    private void HandleDeceleration()
    {
        /*_carRb.AddForceAtPosition(
                (_isBraking ? _brakingDeceleration : _deceleration) * 
                Mathf.Abs(_carVelocityRatio) * 
                -transform.forward.With(y:0), 
            _accelerationPoint.position, 
            ForceMode.Acceleration);*/
        Vector3 forward = transform.forward.With(y: 0).normalized;

        float velocityZ = CurrentCarLocalVelocity.z; // m/s in local space

        // Base decel (coasting vs braking)
        float decel = (_isBraking) ? _brakingDeceleration : 0/*_deceleration*/;

        // Apply braking force opposite to the direction of travel
        Vector3 brakeForce = -forward * (Mathf.Sign(velocityZ) * decel);
        // when sliding downwards but wanting to go in the opposite direction, dont apply force in slipping direction

        CarRb.AddForceAtPosition(brakeForce, _accelerationPoint.position, ForceMode.Acceleration);

        // Anti-roll / handbrake assist
        /*if (Mathf.Abs(_engineRpm - _idleRpm) < 50 && Mathf.Abs(velocityZ) < 0.2f)
        {
            // Counteract slope gravity to keep car still
            Vector3 gravityComp = -Physics.gravity.y * Vector3.down; // world gravity vector
            _carRb.AddForce(-_carRb.linearVelocity / Time.fixedDeltaTime, ForceMode.Acceleration); // hard clamp
            _carRb.linearVelocity = Vector3.zero;
        }*/
        
    }

    /*private void Turn()
    {
        _carRb.AddTorque(
                _steerStrength * 
                _steerInput * 
                _turningCurve.Evaluate(Mathf.Abs(_carVelocityRatio)) * 
                Mathf.Sign(_carVelocityRatio) * transform.up, 
            ForceMode.Acceleration);
    }*/

    private void SidewaysDrag()
    {
        float currentSidewaysSpeed = CurrentCarLocalVelocity.x;

        float dragMagnitude = -currentSidewaysSpeed * (_isBraking ? _brakingDragCoefficient : _dragCoefficient);
        
        Vector3 dragForce = transform.right * dragMagnitude;
        
        CarRb.AddForceAtPosition(dragForce, CarRb.worldCenterOfMass, ForceMode.Acceleration);
    }

    #endregion

    #region WheelFunctions
    
    private void WheelFunctions()
    {
        for (int i = 0; i < _suspensionRayPoints.Length; i++)
        {
            Transform rayPoint = _suspensionRayPoints[i];
            float maxLength = _restLengthFront + _springMaxTravel; // (i < 2 ? _restLengthFront : _restLengthBack)
            if (Physics.Raycast(rayPoint.position, -rayPoint.up, out var hit, 10f, _drivable))
            {
                HandleSuspension(hit, rayPoint);
                if (hit.distance < maxLength + _wheelRadiusFront)
                {
                    HandleAccelerationViaWheels(rayPoint, i);
                    HandleSteeringViaWheels(rayPoint, i);
                    _wheelsIsGrounded[i] = true;
                    SetTirePosition(_tires[i], hit.point + rayPoint.up * _wheelRadiusFront);
                }
                else
                {
                    _wheelsIsGrounded[i] = false;
                    SetTirePosition(_tires[i], rayPoint.position - rayPoint.up * (maxLength - _wheelRadiusFront));
                }
            }
            else
            {
                _wheelsIsGrounded[i] = false;
                
                SetTirePosition(_tires[i], rayPoint.position - rayPoint.up * (maxLength - _wheelRadiusFront));
            }
        }
    }

    private void HandleAccelerationViaWheels(Transform rayPoint, int index)
    {
        if (index < 2) return;

        if (Mathf.Abs(_moveInput) > 0.001f)
        {
            float engineTorque = _torqueCurve.Evaluate(EngineRpm);

            //engineTorque *= Mathf.Clamp01(_moveInput);
            
            //int gearToUse = _currentGear > 0 ? _currentGear - 1 : 0;
            
            int gearToUse = _currentGear switch
            {
                0 => 0,
                _ => Math.Abs(_currentGear) - 1,
            };

            float wheelTorque = engineTorque * _gearRatios[gearToUse] * _finalDriveRatio * _engineTune * Mathf.Sign(_moveInput) * _gearTorqueRatios[gearToUse];

            float driveForce = wheelTorque / _wheelRadiusFront;
            
            Vector3 accelDir = rayPoint.forward;
            CarRb.AddForceAtPosition(accelDir * driveForce, rayPoint.position);

            /*float availableTorque = _engineTorque.Evaluate(Mathf.Abs(_carVelocityRatio)) * _moveInput * _acceleration2;

            _carRb.AddForceAtPosition(_carRb.mass * availableTorque * accelDir, rayPoint.position);*/
        }
    }

    private void HandleSteeringViaWheels(Transform rayPoint, int index)
    {
        Vector3 steeringDir = rayPoint.right;
        Vector3 tireWorldVelocity = CarRb.GetPointVelocity(rayPoint.position);
        float steeringVel = Vector3.Dot(steeringDir, tireWorldVelocity);

        float evaluateValue = Mathf.Abs(steeringVel / _maxSpeedForward);
        
        float tireGrip = Mathf.Clamp01((index < 2
                ? _frontTiresGrip.Evaluate(evaluateValue)
                : _rearTiresGrip.Evaluate(evaluateValue)) * HandlingMultiplier);
        
        DEBUG_GripCurve[index] = evaluateValue;

        float desiredVelChange = -steeringVel * tireGrip * _steerStrength2;
        
        float desiredAcceleration = desiredVelChange * Time.fixedDeltaTime;
        
        CarRb.AddForceAtPosition(steeringDir * (CarRb.mass * desiredAcceleration), rayPoint.position);
    }

    private void HandleSuspension(RaycastHit hit, Transform rayPoint)
    {
        float currentSpringLength = hit.distance - _wheelRadiusFront; //i < 2 ? _wheelRadiusFront : _wheelRadiusBack
        float springCompression = (_restLengthFront - currentSpringLength) / _springMaxTravel; //(i < 2 ? _restLengthFront : _restLengthBack)

        float springVelocity = Vector3.Dot(CarRb.GetPointVelocity(rayPoint.position), rayPoint.up);
        float dampForce = _damperStiffness * springVelocity;
        
        float springForce = _springStiffness * springCompression;

        float netForce = springForce - dampForce;
        
        CarRb.AddForceAtPosition(rayPoint.up * netForce, rayPoint.position);
    }

    #endregion

    #region  Visuals

    private void Visuals()
    {
        TireVisuals();
    }
    
    private void SetTirePosition(GameObject tire, Vector3 targetPosition)
    {
        tire.transform.position = targetPosition;
    }

    private void UiVisuals()
    {
        if (_racePosition)
            _racePosition.text = "" + RacePosition + " / " + _allCars.Length;
        
        float speed = CurrentCarLocalVelocity.z * 3.6f;

        if (_speedometerText)
        {
            _speedometerText.text = $"{(int)speed} km/h";
        }

        if (_speedometerArrow)
        {
            _speedometerArrow.localEulerAngles = new Vector3(
                0, 0,
                Mathf.Lerp(_minMaxSpeedArrowAngle.x, _minMaxSpeedArrowAngle.y, EngineRpm / _maxRpm)
                );
        }

        if (_steerLeft && _steerRight)
        {
            if (_steerInput < 0)
            {
                _steerLeft.fillAmount = Mathf.Abs(_steerInput);
                _steerRight.fillAmount = 0;
            }
            else
            {
                _steerRight.fillAmount = Mathf.Abs(_steerInput);
                _steerLeft.fillAmount = 0;
            }
        }
        
        if (_braking && _gas)
        {
            if (_moveInput < 0)
            {
                _braking.fillAmount = Mathf.Abs(_moveInput);
                _gas.fillAmount = 0;
            }
            else
            {
                _gas.fillAmount = Mathf.Abs(_moveInput);
                _braking.fillAmount = 0;
            }
        }
        
        _currentGearDisplay.text = _currentGear switch
        {
            -1 => "R",
            0 => "N",
            _ => _currentGear.ToString("D")
        };
    }

    private void TireVisuals()
    {
        float steeringAngle = _maxSteeringAngle * _steerInput;
        
        for (int i = 0; i < _tires.Length; i++)
        {
            if (i < 2)
            {
                _tires[i].transform.Rotate(Vector3.right, _tireRotationSpeed * _carVelocityRatio * Time.fixedDeltaTime,
                    Space.Self);

                Vector3 newTireAngle = _frontTireParents[i].transform.localEulerAngles;
                newTireAngle.y = steeringAngle;
                _frontTireParents[i].transform.localEulerAngles = newTireAngle;
            }
            else
            {
                _tires[i].transform.Rotate(Vector3.right, _tireRotationSpeed * _moveInput * Time.fixedDeltaTime,
                    Space.Self);
            }
        }
    }
    #endregion

    #region RPM and Shifting
    
    private void UpdateEngineRpm()
    {
        if (_currentGear == 0) return;
        if (_rpmIsAnimating) return;
        
        _wheelRpm = (Mathf.Abs(CurrentCarLocalVelocity.z) / _wheelCircumference) * 60f;
        
        int gearToUse = _currentGear > 0 ? _currentGear - 1 : 0;

        float targetWheelRpm = _wheelRpm * _gearRatios[gearToUse] * _finalDriveRatio;

        EngineRpm = Mathf.MoveTowards(EngineRpm, targetWheelRpm,
            Time.deltaTime * (!_isBraking ? _throttleRpmIncreaseRate : _throttleRpmDecreaseRate) * MathF.Sqrt(_gearRatios[gearToUse]));
        
        EngineRpm = Mathf.Clamp(EngineRpm, _idleRpm, _maxUsableRpm);
        DEBUG_EngineTorqueCurve = EngineRpm;
    }

    private void ShiftUp()
    {
        if (_currentGear >= _gearRatios.Length) return;
        if (_currentGear <= 0) return;
        if (_rpmIsAnimating) return;
        
        float ratioChange = _gearRatios[_currentGear + 1 - 1] / _gearRatios[_currentGear - 1]; 
        // -1 on both for converting from 0-based (array indices) to 1-based (_currentGear)
        EngineRpm *= ratioChange;
        _currentGear++;
    }
    
    private void ShiftDown()
    {
        if (_currentGear <= 1) return;
        if (_rpmIsAnimating) return;
        
        float ratioChange = _gearRatios[_currentGear - 1 - 1] / _gearRatios[_currentGear - 1];
        // -1 on both for converting from 0-based (array indices) to 1-based (_currentGear)
        EngineRpm *= ratioChange;
        _currentGear--;
    }

    private void CheckAutoShift()
    {
        if (_rpmIsAnimating) return;
        
        // upshifting
        if (EngineRpm > _upshiftRpm && _previousEngineRpm < EngineRpm)
        {
            ShiftUp();
            return;
        }

        // downshifting
        if (EngineRpm < _downshiftRpm && _previousEngineRpm > EngineRpm)
        {
            ShiftDown();
        }
    }

    private void AutoStartAndReverse()
    {
        if (_rpmIsAnimating) return;
        
        bool carIsStopped = Mathf.Abs(CurrentCarLocalVelocity.z) < 0.5f; // less than 0.5 m/s is stopped.
        float velZ = CurrentCarLocalVelocity.z;

        // Neutral to first
        if (_moveInput > 0.1f && _currentGear == 0)
        {
            _currentGear = 1;
            EngineRpm = _idleRpm;
        }

        // Neutral to reverse
        if (_moveInput < -0.1f && _currentGear == 0 && EngineRpm < 50)
        {
            _currentGear = -1;
            EngineRpm = _idleRpm;
        }

        // From reverse to first
        if (_currentGear == -1)
        {
            if (_moveInput > 0.1f &&
                (carIsStopped || // case 1: fully stopped + pressing forward
                velZ > 1.5f))    // case 2: actually moving forward significantly + holding forward
            {
                SetEngineRpmToZeroToIdle(1);
            }
        }

        // From forward to reverse
        if (_currentGear > 0)
        {
            if (_moveInput < -0.1f &&
                (carIsStopped || // case 1: fully stopped + pressing reverse
                velZ < -1.5f))   // case 2: actually moving backward significantly + holding reverse
            {
                SetEngineRpmToZeroToIdle(-1);
            }
        }
        /*if (_moveInput > 0.1f)
        {
            // In neutral, switch to first
            if (_currentGear == 0)
            {
                _currentGear = 1;
                _engineRpm = _idleRpm;
            }

            // From reverse to first 
            if (_currentGear == -1 && carIsStopped )
            {
                _currentGear = 1;                
                SetEngineRpmToZeroToIdle(); // gradually changes from current rpm to zero to idle rpm
            }
        }

        if (_moveInput < -0.1f)
        {
            // In neutral, switch to reverse
            if (_currentGear == 0 && _engineRpm < 50)
            {
                _currentGear = -1;
                _engineRpm = _idleRpm;
            }
            
            // From forward to reverse
            if (_currentGear > 0 && carIsStopped)
            {
                _currentGear = -1;
                SetEngineRpmToZeroToIdle(); // gradually changes from current rpm to zero to idle rpm
            }
        }*/
        
        
        
    }

    private void SetEngineRpmZeroToIdle()
    {
        Tween.Custom(0f, _idleRpm, (_idleRpm - 0) / _throttleRpmIncreaseRate, newVal => EngineRpm = newVal);
    }
    
    private void SetEngineRpmToZero()
    {
        float currentRpm = EngineRpm;
        Tween.Custom(currentRpm, 0, (_idleRpm - 0) / (_throttleRpmDecreaseRate * _gearRatios[0]), newVal => EngineRpm = newVal);
    }

    private void SetEngineRpmToZeroToIdle(int gearToSwitchTo)
    {
        float currentRpm = EngineRpm;

        _rpmIsAnimating = true;
        
        Sequence.Create()
            .Chain(Tween.Custom(currentRpm, 0, (_idleRpm - 0) / (_throttleRpmDecreaseRate * _gearRatios[0]),
                newVal => EngineRpm = newVal))
            .Chain(Tween.Custom(0f, _idleRpm, (_idleRpm - 0) / _throttleRpmIncreaseRate, 
                newVal => EngineRpm = newVal))
            .OnComplete(() =>
            {
                _rpmIsAnimating = false;
                _currentGear = gearToSwitchTo;
            });
    }

    #endregion
}
