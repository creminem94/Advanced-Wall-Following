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
        line.transform.localPosition = new Vector3(0.0f,0.0f,0.35f);
        // line.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
    }

    // Update is called once per frame
    void Update()
    {
    	//Destroy(linePrefab)
    	//playerPosition.y += 1;
      	//GameObject newFurbot = Instantiate(furbot, playerPosition, Quaternion.identity);
      	//Destroy(furbot);
      	//furbot = newFurbot;
       //GameObject clone = (GameObject)Instantiate(linePrefab,transform.position+new Vector3(0.0f,0.0f,0.01f),Quaternion.identity);
       //Destroy (clone, 1.0f);
    }
}
