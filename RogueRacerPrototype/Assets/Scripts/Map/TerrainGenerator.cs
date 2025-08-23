using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;
using UnityUtils;

public class TerrainGenerator : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Terrain _terrain;                        // Terrain to deform
    [SerializeField] private SplineContainer _splineContainer;        // Road spline

    [Header("Road Settings")]
    [SerializeField] private float _roadWidth = 5f;                   // Half-width of the road
    [SerializeField] private float _roadHeight = 0.2f;                // Height to raise terrain for road
    [SerializeField] private float _falloffRadius = 50f;               // Radius beyond road edges for smooth blending
    [SerializeField] private int _resolutionPerSplineMeter = 2;
    [Range(1f,5f)]     
    [SerializeField] private float _exponent = 4;
    [SerializeField] private float _divider = 10;


    [Header("Debug")]
    public bool _autoApplyOnStart = false;


    private float LongitudinalClamp => 1f / _resolutionPerSplineMeter;


    void Start()
    {
        if (!enabled) return;
        
        if (_autoApplyOnStart)
            ApplyRidge();
    }

    public void ApplyRidge()
    {
        float splineSamples = Mathf.CeilToInt(_splineContainer.CalculateLength()) * _resolutionPerSplineMeter;
        if (!_terrain || !_splineContainer)
        {
            Debug.LogWarning("Missing Terrain or SplineContainer!");
            return;
        }

        TerrainData terrainData = _terrain.terrainData;
        int res = terrainData.heightmapResolution;

        // Load heightmap
        float[,] heights = terrainData.GetHeights(0, 0, res, res);

        Vector3 terrainPos = _terrain.transform.position;
        float terrainWidth = terrainData.size.x;
        float terrainHeight = terrainData.size.y;
        float terrainLength = terrainData.size.z;

        // Sample spline
        for (int i = 0; i < splineSamples; i++)
        {
            float t = i / (float)(splineSamples - 1);
            _splineContainer.Evaluate(t, out var worldPos, out var forw, out var up);
            worldPos.y -= _roadHeight;
            Vector3 forward = ((Vector3)forw).normalized;
            Vector3 upVector = ((Vector3)up).normalized;
            Vector3 right = Vector3.Cross(Vector3.up, forward).normalized;

            // Convert spline world position to terrain indices
            float tx = (worldPos.x - terrainPos.x) / terrainWidth * (res - 1);
            float tz = (worldPos.z - terrainPos.z) / terrainLength * (res - 1);
            int terrainCenterX = Mathf.RoundToInt(tx);
            int terrainCenterZ = Mathf.RoundToInt(tz);

            int brushRadius = Mathf.RoundToInt(_falloffRadius / terrainWidth * res);

            // Modify nearby terrain
            for (int x = -brushRadius; x <= brushRadius; x++)
            {
                for (int z = -brushRadius; z <= brushRadius; z++)
                {
                    int hx = terrainCenterX + x;
                    int hz = terrainCenterZ + z;

                    if (hx < 0 || hx >= res || hz < 0 || hz >= res)
                        continue;

                    // World-space distance from spline point
                    float dx = (hx / (float)(res - 1)) * terrainWidth + terrainPos.x;
                    float dz = (hz / (float)(res - 1)) * terrainLength + terrainPos.z;
                    Vector3 terrainWorldPos = new Vector3(dx, 0, dz);

                    Vector3 toSample = terrainWorldPos - ((Vector3)worldPos).With(y:0);
                    float lateralDist = Mathf.Abs(Vector3.Dot(toSample, right));
                    
                    float longitudinalDist = Mathf.Abs(Vector3.Dot(toSample, forward));

                    if (/*lateralDist > _falloffRadius || */longitudinalDist > LongitudinalClamp) continue;

                    // Height falloff (ridge to ground at y=0)
                    float falloff;
                    if (lateralDist <= _roadWidth)
                    {
                        falloff = 1;
                    }
                    else
                    {
                        float outerF = (lateralDist - _roadWidth) / (_falloffRadius - _roadWidth);
                        falloff  = 1 - Mathf.Pow(outerF, _exponent);
                    }

                    float targetHeight = worldPos.y - Mathf.Pow(lateralDist / _divider, _exponent);
                    
                    if (targetHeight <= 0f) continue;

                    // Normalize to terrain height range
                    float normalizedHeight = targetHeight / terrainHeight;

                    // Apply "ridge" rule: keep highest value
                    if (normalizedHeight > heights[hz, hx])
                        heights[hz, hx] = normalizedHeight;
                }
            }
        }

        // Save back
        terrainData.SetHeights(0, 0, heights);
    }
}
