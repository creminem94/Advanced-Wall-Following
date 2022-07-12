using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using rclcs;

public class lineSpawner : MonoBehaviourRosNode
{
    public string NodeName = "lineSpawner";
    public string lineTopic = "ransac_lines";
    public GameObject linePrefab;

    private List<GameObject> lines = new List<GameObject>();

    protected override string nodeName { get { return NodeName; } }

    private Subscription<geometry_msgs.msg.Twist> lineSubscriber;
    protected override void StartRos()
    {
        lineSubscriber = nodeName.node.CreateSubscription<geometry_msgs.msg.Twist>(
            lineTopic,
            spawnLines
        );
    }

    void spawnLines(geometry_msgs.msg.Twist msg) {
        //step 1: delete all lines stored in this.lines
        foreach (var prevLine in lines) {
            Destroy(prevLine);
        }
        lines = new List<GameObject>();
        //TODO: find a msg types that can be used to return line info
        //step 2: spawn new lines and add them to aray
        foreach (var lineData in msg.lines)
        {
            var newLIne = Instantiate(linePrefab) as GameObject;
            // line.transform.parent = gameObject.transform;
            // line.transform.localPosition = new Vector3(0.0f,0.0f,0.35f);
            lines.Add(newLine);
        }
    }

    void Update() {
        SpinSome();
    }

}
