using System;
using System.Collections.Generic;
using UnityEngine;
using Object = UnityEngine.Object;

public class CurveVisualizer : MonoBehaviour
{
    [Range(0,2)]
    [SerializeField] private int _curveToSample;
    [SerializeField] private CarController _carController;
    [SerializeField] private int _sampleCount = 100;
    [SerializeField] private GameObject _pathPointPrefab;
    [SerializeField] private float _prefabSize = 1f;
    [SerializeField] private RectTransform[] _curveIndicators;
    [SerializeField] private bool _trueCurveSize = true;
    
    private RectTransform _rect;
    private List<RectTransform> _pathPoints = new List<RectTransform>();
    private AnimationCurve _curve;
    private Vector2 _curveCorrection;

    private void Start()
    {
        _rect = GetComponent<RectTransform>();
        
        _curve = _curveToSample switch
        {
            0 => _carController._frontTiresGrip,
            1 => _carController._rearTiresGrip,
            2 => _carController._torqueCurve,
            _ => AnimationCurve.Constant(0, 1, 0)
        };
        
        _curveCorrection = Vector2.one;
        if (!_trueCurveSize)
        {
            _curveCorrection *= 0;
            foreach (Keyframe keyframe in _curve.keys)
            {
                if (keyframe.time > _curveCorrection.x)
                {
                    _curveCorrection.x = keyframe.time;
                }

                if (keyframe.value > _curveCorrection.y)
                {
                    _curveCorrection.y = keyframe.value;
                }
            }
        }
                    
        for (int i = 0; i < _sampleCount; i++)
        {
            float t = i / (float)(_sampleCount - 1) * _curveCorrection.x;
            
            float curveHeight = _curve.Evaluate(t);
           
            GameObject dot = Instantiate(_pathPointPrefab, _rect);
            RectTransform dotRect = dot.GetComponent<RectTransform>();
            dotRect.localScale = Vector3.one * _prefabSize;
            
            dotRect.anchoredPosition = new Vector2(
                (t * _rect.rect.width) / _curveCorrection.x + (-_rect.rect.width / 2),
                (-_rect.rect.height / 2) + (curveHeight * _rect.rect.height) / _curveCorrection.y
                );
            
            _pathPoints.Add(dotRect);
        }
    }

    private void Update()
    {
        switch (_curveToSample)
        {
            case 0:
            case 1:
            {
                for (int i = 0; i < _curveIndicators.Length; i++)
                {
                    var indicator = _curveIndicators[i];
                    indicator.anchoredPosition = new Vector2(
                            _carController.DEBUG_GripCurve[i + _curveToSample * 2] * _rect.rect.width / _curveCorrection.x,
                            0
                        );
                    var scale = indicator.localScale;
                    scale.y = _curve.Evaluate(_carController.DEBUG_GripCurve[i + _curveToSample * 2]) / _curveCorrection.y;
                    indicator.localScale = scale;
                }
                break;
            }
            case 2:
            {
                var indicator = _curveIndicators[0];
                indicator.anchoredPosition = new Vector2(
                        _carController.DEBUG_EngineTorqueCurve * _rect.rect.width / _curveCorrection.x,
                        0
                    );
                var scale = indicator.localScale;
                scale.y = _curve.Evaluate(_carController.DEBUG_EngineTorqueCurve) / _curveCorrection.y;
                indicator.localScale = scale;
                break;
            }
        }
    }
}
