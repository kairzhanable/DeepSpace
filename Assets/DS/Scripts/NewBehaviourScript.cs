using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class NewBehaviourScript : MonoBehaviour
{
    public GameObject engine;
    public GameObject slot;
    public ShipEcosystem ship;
    void Start()
    {
        
    }

    void Update()
    {
        if(Input.GetKey(KeyCode.Space)){
            ship.AddModule(engine, slot);
        }
    }
}
