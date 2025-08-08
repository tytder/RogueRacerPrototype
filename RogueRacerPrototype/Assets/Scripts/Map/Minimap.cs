using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

public class Minimap : MonoBehaviour
{
    public SplineContainer splineContainer;
    public int sampleCount = 100;
    public List<Vector3> sampledPoints = new List<Vector3>();
    public RectTransform minimapRect;
    public Vector2 minimapSize = new Vector2(200, 200);
    public GameObject pathPointPrefab;
    public RectTransform playerIcon;
    public Transform player;

    public float minimapScale = 0.1f;

    private List<RectTransform> pathPoints = new List<RectTransform>();

    void Start()
    {
        for (int i = 0; i < sampleCount; i++)
        {
            float t = i / (float)(sampleCount - 1);
            Vector3 worldPos = splineContainer.EvaluatePosition(t);

            GameObject dot = Instantiate(pathPointPrefab, minimapRect);
            RectTransform dotRect = dot.GetComponent<RectTransform>();
            dotRect.anchoredPosition = WorldToMinimap(worldPos);
            pathPoints.Add(dotRect);
        }
        playerIcon.SetAsLastSibling();
    }

    void Update()
    {
        playerIcon.anchoredPosition = WorldToMinimap(player.position);
    }
    
    Vector2 WorldToMinimap(Vector3 worldPos)
    {
        // Optional: define bounds manually or calculate from spline
        Vector3 center = splineContainer.transform.position;
        Vector3 offset = worldPos - center;

        // Scale and flip Y if needed
        return new Vector2(offset.x, offset.z) * minimapScale;
    }
    
    
}
