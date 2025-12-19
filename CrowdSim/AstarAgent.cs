using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using PathFinding;
public class AstarAgent : MonoBehaviour
{
    public Transform AgentT;
    public Transform target;
    public GridManager gridManager;
    // Start is called before the first frame update
    protected Vector3 GridLoc = Vector3.zero;
    protected Vector3 TargetLoc = Vector3.zero;
    protected float halfX = 0;
    protected float halfZ = 0;
    void Start()
    {
        halfX = (gridManager.maxX - gridManager.minX) / 2;
        halfZ = (gridManager.maxZ - gridManager.minZ) / 2;
        GridLoc = AgentT.position;


    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
