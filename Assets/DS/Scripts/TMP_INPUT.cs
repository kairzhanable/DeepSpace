using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DeepSpace;

using Rewired;

[RequireComponent(typeof(CharacterController))]
public class TMP_INPUT : MonoBehaviour
{
    // The Rewired player id of this character
    public int playerId = 0;
    private Player player; // The Rewired Player

    public GameObject Ship;
    public GameObject engine;
    public GameObject camera; 

    private ManeurSystem SAS;
    private List<Slot> slots;
    private Vector3 desired_speed;
    private Vector3 desired_rotation;

    private void Awake() {
        Cursor.visible = false;
        // Get the Rewired Player object for this player and keep it for the duration of the character's lifetime
        player = ReInput.players.GetPlayer(playerId);
    }
    
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
        GetInput();
        ProcessInput();
    }

    private float acceleration_step = 1000;
    private float roll_step = 200;
    private float pitch_step = 500;
    private float yaw_step = 500;

    private void GetInput()
    {
        desired_speed.x += player.GetAxis("RightLeft") * acceleration_step * Time.deltaTime;
        desired_speed.y += player.GetAxis("UpDown") * acceleration_step * Time.deltaTime;
        desired_speed.z += player.GetAxis("ForwardBack") * acceleration_step * Time.deltaTime;
        
        if(player.GetButton("ResetSpeed"))
            desired_speed = Vector3.zero;

        desired_rotation.z = player.GetAxis("Roll") * roll_step * Time.deltaTime;

        desired_rotation.y =  player.GetAxis("Yaw") * yaw_step * Time.deltaTime;
        desired_rotation.x =  player.GetAxis ("Pitch") * pitch_step * Time.deltaTime;
    }

    private void ProcessInput() 
    {
        camera.transform.Rotate(desired_rotation);
        camera.transform.position = Ship.transform.position;
        Quaternion desiredRotation = camera.transform.rotation;

        SAS.ApplyManeur(desired_speed, desiredRotation);
    }
}



