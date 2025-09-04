using System;
using Sirenix.OdinInspector;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Splines;
using UnityUtils;

//[Serializable]
public class Waypoint : MonoBehaviour
{
     private Vector3 _position;
     public Vector3 Position
     {
          get => _position;
          set
          {
               _position = value;
               transform.position = _position;
          } 
     }
     public Vector3 AccelVector;
     public Vector3 TangentVector;

     private Waypoint _nextWaypoint;
     [ReadOnly] public float NextWaypointAccelDelta;

     public Waypoint NextWaypoint
     {
          get => _nextWaypoint;
          set
          {
               _nextWaypoint = value;
               NextWaypointAccelDelta = value.AccelVector.With(y:0).magnitude - AccelVector.With(y:0).magnitude;
          }
     }

     private Waypoint _previousWaypoint;
     [ReadOnly] public float PrevWaypointAccelDelta;

     public Waypoint PreviousWaypoint
     {
          get => _previousWaypoint;
          set
          {
               _previousWaypoint = value;
               PrevWaypointAccelDelta = AccelVector.With(y:0).magnitude - value.AccelVector.With(y:0).magnitude;
          }
     }

     public Vector3 VectorToNextPoint => (NextWaypoint.Position - Position);

     private void OnTriggerEnter(Collider other)
     {
          if (other.transform.parent.TryGetComponent(out CarController carController))
          {
               if (!carController.SetCurrentWaypoint(this))
               {
                    Debug.LogWarning("Targets was not yet set, setting first target for all cars");
                    var cars = FindObjectsByType<CarController>(FindObjectsSortMode.None);
                    foreach (var car in cars)
                         car.SetCurrentWaypoint(this);
               }
               return;
          }
          if (other.transform.parent.parent.TryGetComponent(out CarController carController2))
          { 
               carController2.SetCurrentWaypoint(this);
          }
     }
}
