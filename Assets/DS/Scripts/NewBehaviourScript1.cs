using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

public class NewBehaviourScript1 : MonoBehaviour
{
    
    public GameObject Ship;
    public GameObject engine;
    public GameObject camera; 

    private ManeurSystem SAS;
    private List<Slot> slots;
    private Vector3 desired_speed;

    void Start()
    {
        slots = new List<Slot>();
        slots = Ship.transform.GetComponentsInChildrenRecursively<Slot>(slots);
        SAS = Ship.GetComponent<ManeurSystem>();
        ShipEcosystem main = Ship.GetComponent<ShipEcosystem>();

        foreach(Slot slot in slots){
            if(slot.moduleType == ModuleType.ENGINE){
                GameObject new_engine = Instantiate(engine, Vector3.zero, Quaternion.identity);
                main.AddModule(new_engine, slot);
            }
        }

        engine.active = false;
    }

    void Update()
    {
        float x = 500 * Input.GetAxis ("Mouse X") * Time.deltaTime;
        float y = 500 * -Input.GetAxis ("Mouse Y") * Time.deltaTime;
        float z = 0;
        if(Input.GetKey(KeyCode.Q)){
            z = 200 * Time.deltaTime;
        }
        if(Input.GetKey(KeyCode.E)){
            z = -200 * Time.deltaTime;
        }
        camera.transform.Rotate(y, x, z);
        camera.transform.position = Ship.transform.position;
        Quaternion desiredRotation = camera.transform.rotation;

        if(Input.GetKey(KeyCode.W)){
            desired_speed += Vector3.forward * 1000 * Time.deltaTime;
        }
        if(Input.GetKey(KeyCode.S)){
            desired_speed += Vector3.back * 1000 * Time.deltaTime;
        }
        if(Input.GetKey(KeyCode.D)){
            desired_speed += Vector3.right * 1000 * Time.deltaTime;
        }
        if(Input.GetKey(KeyCode.A)){
            desired_speed += Vector3.left * 1000 * Time.deltaTime;
        }
        if(Input.GetKeyDown(KeyCode.Space)){
            desired_speed = Vector3.zero;
        }
        if(Input.GetKeyDown(KeyCode.UpArrow)){
            desired_speed += Vector3.up * 1000 * Time.deltaTime;
        }
        if(Input.GetKeyDown(KeyCode.DownArrow)){
            desired_speed += Vector3.down * 1000 * Time.deltaTime;
        }

        SAS.ApplyManeur(desired_speed, desiredRotation);
    }
}



