using System.Text;
using UnityEngine;
using UnityUtils;
using Random = UnityEngine.Random;

    public class AICarInputUnity : MonoBehaviour, ICarInput
    {
        // This script provides input to the car controller in the same way that the user control script does.
        // As such, it is really 'driving' the car, with no special physics or animation tricks to make the car behave properly.

        // "wandering" is used to give the cars a more human, less robotic feel. They can waver slightly
        // in speed and direction while driving towards their target.

        [SerializeField] [Range(0, 1)] private float m_CautiousSpeedFactor = 0.05f;               // percentage of max speed to use when being maximally cautious
        [SerializeField] [Range(0, 180)] private float m_CautiousMaxAngle = 50f;                  // angle of approaching corner to treat as warranting maximum caution
        [SerializeField] private float m_CautiousMaxDistance = 100f;                              // distance at which distance-based cautiousness begins
        [SerializeField] private float m_CautiousAngularVelocityFactor = 30f;                     // how cautious the AI should be when considering its own current angular velocity (i.e. easing off acceleration if spinning!)
        [SerializeField] private float m_SteerSensitivity = 0.05f;                                // how sensitively the AI uses steering input to turn to the desired direction
        [SerializeField] private float m_AccelSensitivity = 0.04f;                                // How sensitively the AI uses the accelerator to reach the current desired speed
        [SerializeField] private float m_BrakeSensitivity = 1f;                                   // How sensitively the AI uses the brake to reach the current desired speed
        [SerializeField] private float m_LateralWanderDistance = 3f;                              // how far the car will wander laterally towards its target
        [SerializeField] private float m_LateralWanderSpeed = 0.1f;                               // how fast the lateral wandering will fluctuate
        [SerializeField] [Range(0, 1)] private float m_AccelWanderAmount = 0.1f;                  // how much the cars acceleration will wander
        [SerializeField] private float m_AccelWanderSpeed = 0.1f;                                 // how fast the cars acceleration wandering will fluctuate
        [SerializeField] private Waypoint m_Target;                                              // 'target' the target object to aim for.
        [SerializeField] private float _reverseRayCheckDistance;
        [SerializeField] private int _wallLayer;
        
        private float m_RandomPerlin;             // A random value for the car to base its wander on (so that AI cars don't all wander in the same pattern)
        private CarController m_CarController;    // Reference to actual car controller we are controlling
        private float m_AvoidOtherCarTime;        // time until which to avoid the car we recently collided with
        private float m_AvoidOtherCarSlowdown;    // how much to slow down due to colliding with another car, whilst avoiding
        private float m_AvoidPathOffset;          // direction (-1 or 1) in which to offset path to avoid other car, whilst avoiding
        private Rigidbody m_Rigidbody;
        
        
        private float _steer;
        private float _move;
        private float _prevDistanceBetweenPoints;
        private float _prevSteer;
        
        private float stuckTimer = 0f;
        private bool reversing = false;
        [SerializeField] private float _steerExponent;
        [SerializeField] private float _steeringResponsiveness;
        [SerializeField] private float m_CarAvoidanceDistance = 3f;


        private void Awake()
        {
            // get the car controller reference
            m_CarController = GetComponent<CarController>();

            // give the random perlin a random value
            m_RandomPerlin = Random.value*100;

            m_Rigidbody = GetComponent<Rigidbody>();

            //_maxDistanceBetweenWaypoints = float.MinValue;
            /*var waypoints = FindFirstObjectByType<RaceLines>().Waypoints;
            foreach (var waypoint in waypoints)
            {
                float distance = waypoint.VectorToNextPoint.With(y:0).magnitude;
                if (distance > _maxDistanceBetweenWaypoints)
                    _maxDistanceBetweenWaypoints = distance;
            }*/
        }


        private void FixedUpdate()
        {
            if (!RaceManager.Instance.IsRaceActive) return;
            if (!m_Target) return;
            
            Vector3 dirToMove = (m_Target.Position - transform.position).With(y:0).normalized;

            if (stuckTimer > 0f)
            {
                stuckTimer -= Time.fixedDeltaTime;
            }
            if (
                Mathf.Abs(m_CarController.CurrentCarLocalVelocity.z) < 0.5f && 
                _move > 0 && 
                stuckTimer <= 0f &&
                m_CarController.WaypointCount > 1 &&
                Physics.Raycast(transform.position, transform.forward, _reverseRayCheckDistance)
                )
            {
                Debug.LogWarning("Reset rotation");
                /*if (Physics.Raycast(transform.position, transform.right, _reverseRayCheckDistance))
                {
                    m_Rigidbody.position -= transform.right;
                }
                if (Physics.Raycast(transform.position, -transform.right, _reverseRayCheckDistance))
                {
                    m_Rigidbody.position += transform.right;
                }*/

                m_Rigidbody.rotation = Quaternion.LookRotation(dirToMove);
                m_Rigidbody.position += Vector3.up / 2;
                m_Rigidbody.linearVelocity = Vector3.zero;
                /*if (!Physics.Raycast(transform.position, -transform.up, 1f, LayerMask.NameToLayer("Drivable")))
                {
                    m_Rigidbody.position += transform.forward.With(y:0) * 3;
                    Debug.LogWarning("Stuck ontop of another car, teleporting to the front");
                }*/
                stuckTimer = 0.2f;
                
                /*stuckTimer += Time.fixedDeltaTime;
                if (stuckTimer > .5f) // stuck for >.5 sec
                {
                    // Check if front is blocked
                    if (Physics.Raycast(transform.position, transform.forward, out var hit, _reverseRayCheckDistance))
                    {
                        reversing = true;
                        Debug.LogWarning("reversing");
                        stuckTimer = 0;
                        _move = -1f;
                        _steer = -_steer; // flip steering when reversing
                    }
                }
            }
            else
            {
                stuckTimer = 0;
            }

            if (reversing)
            {

                // Exit reverse mode if clear
                if (!Physics.Raycast(transform.position, transform.forward, _reverseRayCheckDistance))
                {
                    reversing = false;
                    Debug.LogWarning("stopped reversing");
                }

                return;
                */


                /*// check if face first into wall
                Debug.DrawLine(
                    transform.position,
                    transform.position + transform.forward * _reverseRayCheckDistance,
                    Color.red,
                    .1f);
                if (Physics.Raycast(transform.position, transform.forward, _reverseRayCheckDistance))
                {
                    _move = -1;
                    _steer = Vector3.Dot(-transform.right, dirToMove);
                    //Debug.DrawLine(transform.position, transform.position + transform.forward * _reverseRayCheckDistance);
                    return;
                }

                // check if rear first into wall
                Debug.DrawLine(
                    transform.position,
                    transform.position - transform.forward * _reverseRayCheckDistance,
                    Color.blue,
                    .1f);
                if (Physics.Raycast(transform.position, -transform.forward, _reverseRayCheckDistance))
                {
                    _move = .5f;
                    _steer = Vector3.Dot(transform.right, dirToMove);
                    //Debug.DrawLine(transform.position, transform.position - transform.forward * _reverseRayCheckDistance);
                    return;
                }*/

                /*// not perpendicular to the wall
                _move = -1f;
                if (Physics.Raycast(transform.position, transform.right, out var hitRight, 2f))
                {
                    _steer = -1f;
                    return;
                }

                _steer = 1f;
                return;*/
            }
            
            Vector3 fwd = transform.forward;
            if (m_CarController.CurrentCarLocalVelocity.z > 1f)
            {
                fwd = m_Rigidbody.linearVelocity;
            }

            float desiredSpeed = m_CarController._maxSpeedForward * .8f;
            
            // the car will brake according to the upcoming change in direction of the target. Useful for route-based AI, slowing for corners.

            // check out the angle of our target compared to the current direction of the car
            float approachingCornerAngle = Vector3.Angle(m_Target.transform.forward, fwd);

            // also consider the current amount we're turning, multiplied up and then compared in the same way as an upcoming corner angle
            float spinningAngle = m_Rigidbody.angularVelocity.magnitude*m_CautiousAngularVelocityFactor;

            // if it's different to our current angle, we need to be cautious (i.e. slow down) a certain amount
            float cautiousnessRequired = Mathf.InverseLerp(0, m_CautiousMaxAngle,
                                                           Mathf.Max(spinningAngle,
                                                                     approachingCornerAngle));
            StringBuilder debugText = new StringBuilder();
            debugText.AppendLine("cautiousnessRequired: " + cautiousnessRequired);
            desiredSpeed = Mathf.Lerp(m_CarController._maxSpeedForward*.8f,
                m_CarController._maxSpeedForward * m_CautiousSpeedFactor*.8f,
                                      cautiousnessRequired);
            debugText.AppendLine("DesiredSpeed: " + desiredSpeed);
            

            // Evasive action due to collision with other cars:

            // our target position starts off as the 'real' target position
            Vector3 offsetTargetPos = m_Target.Position;

            // if are we currently taking evasive action to prevent being stuck against another car:
            if (Time.time < m_AvoidOtherCarTime)
            {
                // slow down if necessary (if we were behind the other car when collision occured)
                desiredSpeed *= m_AvoidOtherCarSlowdown;

                // and veer towards the side of our path-to-target that is away from the other car
                offsetTargetPos += m_Target.transform.right * m_AvoidPathOffset;
            }
            else
            {
                // no need for evasive action, we can just wander across the path-to-target in a random way,
                // which can help prevent AI from seeming too uniform and robotic in their driving
                offsetTargetPos += m_Target.transform.right * 
                                   ((Mathf.PerlinNoise(Time.time*m_LateralWanderSpeed, m_RandomPerlin)*2 - 1) * 
                                    m_LateralWanderDistance);
            }

            // use different sensitivity depending on whether accelerating or braking:
            float accelBrakeSensitivity = (desiredSpeed < m_CarController.CurrentCarLocalVelocity.z)
                                              ? m_BrakeSensitivity
                                              : m_AccelSensitivity;

            // decide the actual amount of accel/brake input to achieve desired speed.
            float accel = Mathf.Clamp((desiredSpeed - m_CarController.CurrentCarLocalVelocity.z) *
                                      accelBrakeSensitivity, -1, 1);
            debugText.AppendLine("Desired speed - current speed: " + (desiredSpeed - m_CarController.CurrentCarLocalVelocity.z));

            // add acceleration 'wander', which also prevents AI from seeming too uniform and robotic in their driving
            // i.e. increasing the accel wander amount can introduce jostling and bumps between AI cars in a race
            accel *= (1 - m_AccelWanderAmount) +
                     (Mathf.PerlinNoise(Time.time * m_AccelWanderSpeed, m_RandomPerlin) * m_AccelWanderAmount);

            // calculate the local-relative position of the target, to steer towards
            Vector3 localTarget = transform.InverseTransformPoint(offsetTargetPos);

            // work out the local angle towards the target
            float targetAngle = Mathf.Atan2(localTarget.x, localTarget.z)*Mathf.Rad2Deg;
            float steerFactor = Mathf.Pow(Mathf.Abs(targetAngle) / m_CautiousMaxAngle, _steerExponent);
            debugText.AppendLine("Target angle: " + targetAngle);
            //debugText.AppendLine("Approaching corner / max: " + ((approachingCornerAngle / m_CautiousMaxAngle)+0.25f));

            // get the amount of steering needed to aim the car towards the target
            float targetSteer = Mathf.Clamp(
                targetAngle * m_SteerSensitivity 
                            * Mathf.Clamp01((approachingCornerAngle / m_CautiousMaxAngle)+0.5f)
                            //* steerFactor
                , -1, 1) * Mathf.Sign(m_CarController.CurrentCarLocalVelocity.z);



            // feed input to the car controller.
            //m_CarController.Move(steer, accel, accel, 0f);
            _steer = _steeringResponsiveness <= 0.001f ? 
                targetSteer : 
                Mathf.Lerp(_steer, targetSteer, Time.fixedDeltaTime * _steeringResponsiveness);
            //_prevSteer = currentSteer;
            _move = accel;
            /*float currentDistanceDelta =
                m_Target.PreviousWaypoint.VectorToNextPoint.With(y: 0).magnitude -
                m_Target.VectorToNextPoint.With(y: 0).magnitude;
            debugText.AppendLine("DistanceDelta: " + (currentDistanceDelta - _prevDistanceBetweenPoints));

            _move = Mathf.Sign(currentDistanceDelta - _prevDistanceBetweenPoints + 0.05f);
            _prevDistanceBetweenPoints = currentDistanceDelta;
            */
            
            //Debug.Log(debugText.ToString());
        }


        private void OnCollisionStay(Collision col)
        {
            // detect collision against other cars, so that we can take evasive action
            if (col.rigidbody != null)
            {
                var otherAI = col.rigidbody.GetComponent<CarController>();
                if (otherAI != null)
                {
                    // we'll take evasive action for 1 second
                    m_AvoidOtherCarTime = Time.time + 1;

                    // but who's in front?...
                    if (Vector3.Angle(transform.forward, otherAI.transform.position - transform.position) < 90)
                    {
                        // the other ai is in front, so it is only good manners that we ought to brake...
                        m_AvoidOtherCarSlowdown = 0.5f;
                    }
                    else
                    {
                        // we're in front! ain't slowing down for anybody...
                        m_AvoidOtherCarSlowdown = 1;
                    }

                    // both cars should take evasive action by driving along an offset from the path centre,
                    // away from the other car
                    var otherCarLocalDelta = transform.InverseTransformPoint(otherAI.transform.position);
                    float otherCarAngle = Mathf.Atan2(otherCarLocalDelta.x, otherCarLocalDelta.z);
                    m_AvoidPathOffset = m_CarAvoidanceDistance*-Mathf.Sign(otherCarAngle);
                }
            }
        }

        public float Move => _move;
        public float Steer => _steer;
        public bool SetCurrentWaypoint(Waypoint currentWaypoint) 
        {
            bool ret = m_Target;
            m_Target = currentWaypoint.NextWaypoint;
            return ret;
        }
    }
