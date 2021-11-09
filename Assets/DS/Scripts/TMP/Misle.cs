using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Misle : MonoBehaviour
{
    public GameObject target;
    public GameObject marker;
    private Rigidbody rigidbody;
    private Rigidbody target_rigidbody;

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = gameObject.GetComponent<Rigidbody>();
        target_rigidbody = target.GetComponent<Rigidbody>();
        target_rigidbody.AddForce(new Vector3(50,10,10), ForceMode.Impulse);
        //rigidbody.AddRelativeForce(Vector3.forward * 35, ForceMode.Impulse);
    }

    // Update is called once per frame
    void Update()
    {
        target_rigidbody.AddForce(new Vector3(0,5f,0), ForceMode.Force);
        rigidbody.velocity = (rigidbody.velocity.magnitude + accel * Time.deltaTime) * gameObject.transform.forward;
        Vector3 point = optimize();
        marker.transform.position = point;
        gameObject.transform.LookAt(marker.transform);
    }

    Vector3 current_target_pos;
    Vector3 current_pos;

    Vector3 target_vel;
    Vector3 vel;

    private Vector3 optimize(){

        target_vel = target_rigidbody.velocity;
        vel = rigidbody.velocity;

        current_target_pos = target_rigidbody.transform.position;
        current_pos = rigidbody.transform.position;

        float t = 0;
        float step = 1000f;
        float error = float.MaxValue;

        for(int iteration = 0; iteration < 2000; iteration++){
            float error_up = calc_error(t + step);
            float error_down = calc_error(t - step);

            if(error_up < error && error_up < error_down){
                t += step; 
                error = error_up;
            } else if(error_down < error && error_down < error_up){
                t -= step;
                error = error_down;
            } else {
                step /=2;
            }
        }

        /*if(t < 0.1)
            return current_pos + gameObject.transform.forward * 10;*/

        return current_target_pos + target_vel * t;
    }

    float accel = 50;

    private float calc_error(float t){
        if(t <= 0)
            return float.MaxValue;
        Vector3 target_pos = current_target_pos + target_vel * t;
        float mean_vel = vel.magnitude + (accel * t) / 2f;
        float time_to_point = (target_pos - current_pos).magnitude / mean_vel;
        return Mathf.Abs(time_to_point - t);
    }
}
