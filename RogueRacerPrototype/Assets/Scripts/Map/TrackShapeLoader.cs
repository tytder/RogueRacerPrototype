using UnityEngine;
using UnityEngine.Splines;

public class TrackShapeLoader : MonoBehaviour
{
    [SerializeField] private GameObject TrackShapePrefab;

    
    private void Awake()
    {
        if (!enabled) return;
        var generatedTrack = GameObject.FindGameObjectWithTag("GeneratedTrack");
        var generatedTrackContainer = generatedTrack.GetComponent<SplineContainer>();
        
        GameObject trackShapeObj = Instantiate(TrackShapePrefab, generatedTrack.transform.position, Quaternion.identity);
        var trackShapeExtrude = trackShapeObj.GetComponent<SplineExtrude>();
        
        trackShapeExtrude.Container = generatedTrackContainer;
        trackShapeExtrude.Rebuild();
        
    }

}
