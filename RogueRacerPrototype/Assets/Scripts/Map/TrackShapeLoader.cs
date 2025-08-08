using System;
using UnityEditor.Splines;
using UnityEngine;
using UnityEngine.Splines;

public class TrackShapeLoader : MonoBehaviour
{
    [SerializeField] private GameObject TrackShapePrefab;
    [SerializeField] private SplineExtrude TrackSpline;

    private SplineContainer _trackShape;
    
    private void Awake()
    {
        GameObject trackShapeObj = Instantiate(TrackShapePrefab);
        _trackShape = trackShapeObj.GetComponent<SplineContainer>();
    
    }

}
