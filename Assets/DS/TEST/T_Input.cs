using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class T_Input : MonoBehaviour
{
    public RCS rcs;

    void Start()
    {
        
    }

    void Update()
    {
        Vector3 force = new Vector3(0,0,0);
        Vector3 moment = new Vector3(0,0,0);

        if (Input.GetKey(KeyCode.W))
        {
           force += new Vector3(0,0,1);
        }
        if (Input.GetKey(KeyCode.S))
        {
           force += new Vector3(0,0,-1);
        }
        if (Input.GetKey(KeyCode.D))
        {
           force += new Vector3(1,0,0);
        }
        if (Input.GetKey(KeyCode.A))
        {
           force += new Vector3(-1,0,0);
        }




        if (Input.GetKey(KeyCode.E))
        {
           moment += new Vector3(0,0,-1);
        }
        if (Input.GetKey(KeyCode.Q))
        {
           moment += new Vector3(0,0,1);
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
           moment += new Vector3(1,0,0);
        }
        if (Input.GetKey(KeyCode.DownArrow))
        {
           moment += new Vector3(-1,0,0);
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
           moment += new Vector3(0,-1,0);
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
           moment += new Vector3(0,1,0);
        }

        rcs.desiredForce = force;
        rcs.desiredTorque = moment;

    }
}
