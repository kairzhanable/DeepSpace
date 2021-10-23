using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Thruster : MonoBehaviour
{

    public float maxTrust = 0;
    public GameObject flame;
    public Vector3 force;
    public Vector3 torque;

    private float tValue = 0;
    
    public void Recalculation(Rigidbody rb)
    {
        Vector3 direction = gameObject.transform.forward;
        Vector3 position = gameObject.transform.position - rb.transform.position + rb.transform.rotation * rb.centerOfMass;
        force = -direction * maxTrust;
        torque = Vector3.Cross(position, -direction * maxTrust);
    }

    public void addForce(Rigidbody rb, float coef) {
        UpdateScale(coef);
        Vector3 direction = gameObject.transform.forward;
        Vector3 position = gameObject.transform.position;
        Vector3 force = -direction * (maxTrust * coef);
        rb.AddForceAtPosition(force, position);
    }

    public void UpdateScale(float scale)
    {
        if (scale < 0)
            scale = 0;
        float alpfa = 0.1f;
        tValue = tValue * (1-alpfa) + scale * alpfa;

        flame.transform.localScale = new Vector3(flame.transform.localScale.x,
                                                flame.transform.localScale.y,
                                                tValue*1000);
    }
}
