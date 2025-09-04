using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Splines;
using UnityEngine.UI;

public class Minimap : MonoBehaviour
{
    public int sampleCount = 100;
    public GameObject pathPointPrefab;
    public GameObject playerIconPrefab;
    public float minimapScale = 0.1f;

    private RectTransform _minimapRect;
    private Transform _player;
    private List<Transform> _aiCars;
    private RectTransform _playerIcon;
    private List<RectTransform> _aiIcons;
    private List<Vector3> _sampledPoints = new List<Vector3>();
    private List<RectTransform> _pathPoints = new List<RectTransform>();
    private Vector2 _minimapOffset;

    void Start()
    {
        var splineContainer = GameObject.FindGameObjectWithTag("GeneratedTrack").GetComponent<SplineContainer>();
        _aiIcons = new List<RectTransform>();

        var cars = FindObjectsByType<CarController>(FindObjectsSortMode.None);
        _player = cars.FirstOrDefault(car => car.PlayerInput != null)?.transform;
        if (_player)
            _playerIcon = Instantiate(playerIconPrefab, transform).GetComponent<RectTransform>();
        _minimapRect = GetComponent<RectTransform>();
        _aiCars = cars.Select(cc => cc.transform).ToList();
        if (!_aiCars.Remove(_player) && _player)
        {
            Debug.LogError("Player not found in car list");
        }

        for (int i = 0; i < _aiCars.Count; i++)
        {
            var dot = Instantiate(playerIconPrefab, transform).GetComponent<RectTransform>();
            dot.localScale *= .85f;
            dot.GetComponent<Image>().color =  Color.black;
            _aiIcons.Add(dot);
        }
        
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

        foreach (var ai in _aiIcons)
        {
            ai.SetAsLastSibling();
        }
        _playerIcon.SetAsLastSibling();
    }


    void Update()
    {
        if (_player)
            _playerIcon.anchoredPosition = WorldToMinimap(_player.position);
        for (var index = 0; index < _aiIcons.Count; index++)
        {
            var aiIcon = _aiIcons[index];
            var aiCar = _aiCars[index];
            aiIcon.anchoredPosition = WorldToMinimap(aiCar.position);
        }
    }
    
    Vector2 WorldToMinimap(Vector3 worldPos)
    {
        Vector2 offset = new Vector2(worldPos.x, worldPos.z) - _minimapOffset;
        return offset * minimapScale;
    }
    
    
}
