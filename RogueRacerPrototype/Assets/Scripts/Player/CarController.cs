using System.Linq;
using Sirenix.OdinInspector;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;
using UnityUtils;

public class CarController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private InputReader _inputReader;
    [SerializeField] private Rigidbody _carRb;
    [SerializeField] private Transform[] _suspensionRayPoints;
    [SerializeField] private LayerMask _drivable;
    [SerializeField] private Transform _accelerationPoint;
    [SerializeField] private GameObject[] _tires = new  GameObject[4];
    [SerializeField] private GameObject[] _frontTireParents = new  GameObject[2];
    
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
    [SerializeField] private AnimationCurve _frontTiresGrip;
    [Tooltip("x-axis: % of tire velocity in perpendicular direction,\n y-axis: % of traction")] 
    [SerializeField] private AnimationCurve _rearTiresGrip;

    [SerializeField] private float _steerStrength = 15f;
    [SerializeField] private float _steerStrength2 = 15f;
    [SerializeField] private AnimationCurve _turningCurve;
    [SerializeField] private float _dragCoefficient = 1f;

    private float _steerInput;
    
    [Header("Acceleration")]
    [Tooltip("x-axis: car's speed as % of top speed,\n y-axis: % of available torque")]
    [SerializeField] private AnimationCurve _engineTorque;
    [SerializeField] private float _maxSpeed = 100f;
    [SerializeField] private float _acceleration = 25f;
    [SerializeField] private float _acceleration2 = 5f;
    [SerializeField] private float _deceleration = 10f;
    [SerializeField] private float _brakingDeceleration = 100f;
    [SerializeField] private float _brakingDragCoefficient = .5f;
    
    private Vector3 _currentCarLocalVelocity;
    private float _carVelocityRatio;
    private float _moveInput;
    
    [Header("Visuals")]
    [SerializeField] private float _tireRotationSpeed = 3000f;
    [SerializeField] private float _maxSteeringAngle = 30f;
    [SerializeField] private Image _steerLeft;
    [SerializeField] private Image _steerRight;
    [SerializeField] private Image _braking;
    [SerializeField] private Image _gas;
    [SerializeField] private TMP_Text _speedometerText;
    [SerializeField] private Vector2 _minMaxSpeedArrowAngle;
    [SerializeField] private RectTransform _speedometerArrow;
    
    private void Start()
    {
        _damperStiffness = 2 * _damperZeta * Mathf.Sqrt(_springStiffness * _carRb.mass);
        _carRb.angularDamping = 10;
    }

    private void Update()
    {
        _moveInput = _inputReader.Move;
        _steerInput = _inputReader.Steer;
    }

    private void FixedUpdate()
    {
        CalculateCarVelocity();
        WheelFunctions();
        Movement();
        TireVisuals();
        UiVisuals();
    }

    private void UiVisuals()
    {
        float speed = _currentCarLocalVelocity.z * 3.6f;

        if (_speedometerText)
        {
            _speedometerText.text = $"{(int)speed} km/h";
        }

        if (_speedometerArrow)
        {
            _speedometerArrow.localEulerAngles = new Vector3(
                0, 0,
                Mathf.Lerp(_minMaxSpeedArrowAngle.x, _minMaxSpeedArrowAngle.y, _carVelocityRatio)
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
    }

    private void Movement()
    {
        HandleDeceleration();
        if (IsGrounded)
        {
            //HandleAcceleration();
            Turn();
            SidewaysDrag();
        }
    }

    private void HandleAcceleration()
    {
        float availableTorque = _engineTorque.Evaluate(_carVelocityRatio) * _moveInput * _acceleration;
        if (_currentCarLocalVelocity.z < _maxSpeed)
        {
            _carRb.AddForceAtPosition(availableTorque * transform.forward.With(y: 0), _accelerationPoint.position,
                ForceMode.Acceleration);
        }
    }

    private void HandleDeceleration()
    {
        _carRb.AddForceAtPosition(
                (/*_inputReader.IsBreaking ? _brakingDeceleration : */_deceleration) * 
                Mathf.Abs(_carVelocityRatio) * 
                -transform.forward.With(y:0), 
            _accelerationPoint.position, 
            ForceMode.Acceleration);
    }

    private void Turn()
    {
        _carRb.AddTorque(
                _steerStrength * 
                _steerInput * 
                _turningCurve.Evaluate(Mathf.Abs(_carVelocityRatio)) * 
                Mathf.Sign(_carVelocityRatio) * transform.up, 
            ForceMode.Acceleration);
    }

    private void SidewaysDrag()
    {
        float currentSidewaysSpeed = _currentCarLocalVelocity.x;

        float dragMagnitude = -currentSidewaysSpeed * (_inputReader.IsBreaking ? _brakingDragCoefficient : _dragCoefficient);
        
        Vector3 dragForce = transform.right * dragMagnitude;
        
        _carRb.AddForceAtPosition(dragForce, _carRb.worldCenterOfMass, ForceMode.Acceleration);
    }

    private void CalculateCarVelocity()
    {
        _currentCarLocalVelocity = transform.InverseTransformDirection(_carRb.linearVelocity);
        _carVelocityRatio = _currentCarLocalVelocity.z / _maxSpeed;
    }

    private void WheelFunctions()
    {
        for (int i = 0; i < _suspensionRayPoints.Length; i++)
        {
            Transform rayPoint = _suspensionRayPoints[i];
            float maxLength = _restLengthFront + _springMaxTravel; // (i < 2 ? _restLengthFront : _restLengthBack)
            if (Physics.Raycast(rayPoint.position, -rayPoint.up, out var hit, maxLength + _wheelRadiusFront, _drivable))
            {
                HandleSuspension(hit, rayPoint, i);
                //HandleSteeringViaWheels(rayPoint, i);
                HandleAccelerationViaWheels(rayPoint);
                _wheelsIsGrounded[i] = true;
            }
            else
            {
                _wheelsIsGrounded[i] = false;
                
                SetTirePosition(_tires[i], rayPoint.position - rayPoint.up * (maxLength - _wheelRadiusFront));
            }
        }
    }

    private void HandleAccelerationViaWheels(Transform rayPoint)
    {
        Vector3 accelDir = rayPoint.forward;

        if (Mathf.Abs(_moveInput) > 0.001f)
        {
            float availableTorque = _engineTorque.Evaluate(_carVelocityRatio) * _moveInput * _acceleration2;
            
            _carRb.AddForceAtPosition(_carRb.mass * availableTorque * accelDir, rayPoint.position);
        }
    }

    private void HandleSteeringViaWheels(Transform rayPoint, int index)
    {
        Vector3 steeringDir = rayPoint.right;
        float steeringVel = _currentCarLocalVelocity.x;

        float tireGrip = Mathf.Clamp01( index < 2
                ? _frontTiresGrip.Evaluate(steeringVel / _maxSpeed)
                : _rearTiresGrip.Evaluate(steeringVel / _maxSpeed));

        float desiredVelChange = -steeringVel * tireGrip * _steerStrength2;
        
        float desiredAcceleration = desiredVelChange / Time.fixedDeltaTime;

        float tireMass = 10.5f;
        
        _carRb.AddForceAtPosition(steeringDir * (tireMass * desiredAcceleration * _steerInput), rayPoint.position);
    }

    private void HandleSuspension(RaycastHit hit, Transform rayPoint, int i)
    {
        float currentSpringLength = hit.distance - _wheelRadiusFront; //i < 2 ? _wheelRadiusFront : _wheelRadiusBack
        float springCompression = (_restLengthFront - currentSpringLength) / _springMaxTravel; //(i < 2 ? _restLengthFront : _restLengthBack)

        float springVelocity = Vector3.Dot(_carRb.GetPointVelocity(rayPoint.position), rayPoint.up);
        float dampForce = _damperStiffness * springVelocity;
        
        float springForce = _springStiffness * springCompression;

        float netForce = springForce - dampForce;
        
        _carRb.AddForceAtPosition(rayPoint.up * netForce, rayPoint.position);
        
        SetTirePosition(_tires[i], hit.point + rayPoint.up * _wheelRadiusFront);
    }

    private void SetTirePosition(GameObject tire, Vector3 targetPosition)
    {
        tire.transform.position = targetPosition;
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
}
