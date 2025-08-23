using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

public class Minimap : MonoBehaviour
{
    public int sampleCount = 100;
    public GameObject pathPointPrefab;
    public GameObject playerIconPrefab;
    public float minimapScale = 0.1f;

    private RectTransform _minimapRect;
    private Transform _player;
    private RectTransform _playerIcon;
    private List<Vector3> _sampledPoints = new List<Vector3>();
    private List<RectTransform> _pathPoints = new List<RectTransform>();
    private Vector2 _minimapOffset;

    void Start()
    {
        var splineContainer = GameObject.FindGameObjectWithTag("GeneratedTrack").GetComponent<SplineContainer>();
        _playerIcon = Instantiate(playerIconPrefab, transform).GetComponent<RectTransform>();
        
        _player = GameObject.FindGameObjectWithTag("Player").transform;
        
        _minimapRect = GetComponent<RectTransform>();
        
        Vector2 minPos = Vector2.one * float.MaxValue;
        Vector2 maxPos = Vector2.one * float.MinValue;
        
        for (int i = 0; i < sampleCount; i++)
        {
            float t = i / (float)(sampleCount - 1);
            Vector3 worldPos = splineContainer.EvaluatePosition(t);

            minPos.x = Mathf.Min(minPos.x, worldPos.x);
            minPos.y = Mathf.Min(minPos.y, worldPos.z);
            maxPos.x = Mathf.Max(maxPos.x, worldPos.x);
            maxPos.y = Mathf.Max(maxPos.y, worldPos.z);
            
            _sampledPoints.Add(worldPos);
        }
        
        _minimapOffset = (minPos + maxPos) / 2;
        
        foreach (Vector3 worldPos in _sampledPoints)
        {
            GameObject dot = Instantiate(pathPointPrefab, _minimapRect);
            RectTransform dotRect = dot.GetComponent<RectTransform>();
            dotRect.anchoredPosition = WorldToMinimap(worldPos);
            _pathPoints.Add(dotRect);
        }
        _playerIcon.SetAsLastSibling();
    }


    void Update()
    {
        _playerIcon.anchoredPosition = WorldToMinimap(_player.position);
    }
    
    Vector2 WorldToMinimap(Vector3 worldPos)
    {
        Vector2 offset = new Vector2(worldPos.x, worldPos.z) - _minimapOffset;
        return offset * minimapScale;
    }
    
    
}
