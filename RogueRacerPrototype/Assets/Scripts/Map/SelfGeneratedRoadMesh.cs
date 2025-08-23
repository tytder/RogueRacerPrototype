using System;
using System.Collections.Generic;
using System.Linq;
using Sirenix.OdinInspector;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;
using UnityUtils;

public class SelfGeneratedRoadMesh : MonoBehaviour
{
    [Serializable]
    private class RoadObjectData
    {
        public string Name;
        public Material Material;
    }
    public SplineContainer SplineContainer;
    [SerializeField] private RoadObjectData[] _roadObjects;
    [SerializeField] private int _resolutionPerMeter;
    public float RoadWidth;

    [Button("Regenerate mesh")]
    private void Awake()
    {
        if (!enabled) return;

        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            DestroyImmediate(child.gameObject);
        }
        
        int totalResolution = Mathf.CeilToInt(_resolutionPerMeter * SplineContainer.CalculateLength(0));
        
        List<Vector3> leftVerts = new List<Vector3>();
        List<Vector3> rightVerts = new List<Vector3>();
        
        float step = 1f / totalResolution;
        for (int i = 0; i < totalResolution; i++)
        {
            float t = i * step;
            SplineContainer.Evaluate(0, t, out var pos, out var forward, out var upVector);
            
            // tangent is the forward direction of travel along the spline to the next point.
            // find the right direction based on this forward direction.
            float3 right = Vector3.Cross(forward, upVector).normalized;
            var p1 = pos + (right * RoadWidth) - (float3)transform.position;
            var p2 = pos + (-right * RoadWidth) - (float3)transform.position;
            rightVerts.Add(p1);
            leftVerts.Add(p2);
        }
        
        List<Vector3> leftVertsLow = leftVerts.Select(p => p - Vector3.up).ToList();
        List<Vector3> leftVertsHigh = leftVerts.Select(p => p + Vector3.up).ToList();
        
        List<Vector3> rightVertsLow = rightVerts.Select(p => p - Vector3.up).ToList();
        List<Vector3> rightVertsHigh = rightVerts.Select(p => p + Vector3.up).ToList();
        
        MeshFilter leftWall = PrepareRoadGameObject(_roadObjects[0]);
        BuildMesh(leftWall, leftVertsLow, leftVertsHigh);
        
        MeshFilter rightWall = PrepareRoadGameObject(_roadObjects[1]);
        BuildMesh(rightWall, rightVertsHigh, rightVertsLow);

        if (_roadObjects.Length >= 3)
        {
            MeshFilter road = PrepareRoadGameObject(_roadObjects[2]);
            BuildMesh(road, rightVerts, leftVerts);
        }
    }

    private MeshFilter PrepareRoadGameObject(RoadObjectData roadObjectData)
    {
        GameObject roadObject =  new GameObject(roadObjectData.Name);
        roadObject.transform.SetParent(transform);
        roadObject.transform.localPosition = Vector3.zero;
        roadObject.transform.localRotation = Quaternion.identity;
        MeshFilter roadMesh = roadObject.AddComponent<MeshFilter>();
        roadObject.AddComponent<MeshRenderer>().material = roadObjectData.Material;
        roadObject.layer = transform.gameObject.layer;
        return roadMesh;
    }

    private void BuildMesh(MeshFilter mesh, List<Vector3> firstList,  List<Vector3> secondList)
    {
        Mesh m = new Mesh();
        List<Vector3> verts = new List<Vector3>();
        List<int> tris = new List<int>();
        int offset = 0;
        
        int length = firstList.Count;
        
        // Iterate verts and build a face
        for (int i = 1; i <= length; i++)
        {
            Vector3 p1 = firstList[i - 1];
            Vector3 p2 = secondList[i - 1];
            Vector3 p3;
            Vector3 p4;

            if (i == length)
            {
                p3 = firstList[0];
                p4 = secondList[0];
            }
            else
            {
                p3 = firstList[i];
                p4 = secondList[i];
            }

            offset = 4 * (i - 1);

            int t1 = offset + 0;
            int t2 = offset + 2;
            int t3 = offset + 3;
            
            int t4 = offset + 3;
            int t5 = offset + 1;
            int t6 = offset + 0;
            
            verts.AddRange(new Vector3[] {p1, p2, p3, p4});
            tris.AddRange(new int[] {t1, t2, t3, t4, t5, t6});
        }
        
        m.SetVertices(verts);
        m.SetTriangles(tris, 0);
        mesh.mesh = m;
        mesh.gameObject.AddComponent<MeshCollider>().sharedMesh = m;
    }
}
