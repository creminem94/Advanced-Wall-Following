using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class lineSpawner : MonoBehaviour
{
    public GameObject linePrefab;
    // Start is called before the first frame update
    void Start()
    {
        GameObject line = Instantiate(linePrefab) as GameObject;
        line.transform.parent = gameObject.transform;
        line.transform.localPosition = new Vector3(0.0f,0.0f,0.0f);
        // line.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}