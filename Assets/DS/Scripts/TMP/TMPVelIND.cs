using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TMPVelIND : MonoBehaviour
{
    public GameObject ship;
    private Rigidbody rigidbody;

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = ship.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        if(rigidbody.velocity.sqrMagnitude != 0)
            gameObject.transform.rotation = Quaternion.LookRotation(rigidbody.velocity, Vector3.up);
    }
}
