using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI4 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

        private new Dictionary<string, Vector3> relative_positions;
        private new Dictionary<string, List<Vector3>> wanted_positions;
        private float INF = 999999999.0f;
        List<GameObject> walls = new List<GameObject>();

        public Vector3 get_pos(Vector3Int pos)
        {
            float step_x = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float x = terrain_manager.myInfo.x_low + step_x / 2 + step_x * pos.x;
            float step_z = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            float z = terrain_manager.myInfo.z_low + step_z / 2 + step_z * pos.z;
            return new Vector3(x, 0.0f, z);
        }

        public bool is_in_wall(Vector3 pos)
        {
            foreach (GameObject wall in walls)
            {
                Vector3 tl = wall.transform.position + new Vector3(-wall.transform.localScale.x, 0.0f, -wall.transform.localScale.z);
                Vector3 tr = wall.transform.position + new Vector3(-wall.transform.localScale.x, 0.0f, wall.transform.localScale.z);
                Vector3 bl = wall.transform.position + new Vector3(wall.transform.localScale.x, 0.0f, -wall.transform.localScale.z);
                Vector3 br = wall.transform.position + new Vector3(wall.transform.localScale.x, 0.0f, wall.transform.localScale.z);

                if (pos.x > wall.transform.position.x - 1.1f * wall.transform.localScale.x &&
                    pos.x < wall.transform.position.x + 1.1f * wall.transform.localScale.x &&
                    pos.z > wall.transform.position.z - 0.9f * wall.transform.localScale.z &&
                    pos.z < wall.transform.position.z + 0.9f * wall.transform.localScale.z)
                {
                    return true;
                }
            }

            return false;
        }

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            // Note that you are not allowed to check the positions of the turrets in this problem

            relative_positions = new Dictionary<string, Vector3>();
            relative_positions.Add("ArmedCar (2)", new Vector3(-40.0f, 0.0f, -10.0f));
            relative_positions.Add("ArmedCar (3)", new Vector3(-10.0f, 0.0f, -15.0f));
            relative_positions.Add("ArmedCar (4)", new Vector3(10.0f, 0.0f, -20.0f));
            relative_positions.Add("ArmedCar (5)", new Vector3(40.0f, 0.0f, -25.0f));
            
            wanted_positions = new Dictionary<string, List<Vector3>>();
            wanted_positions.Add("ArmedCar (2)", new List<Vector3>());
            wanted_positions.Add("ArmedCar (3)", new List<Vector3>());
            wanted_positions.Add("ArmedCar (4)", new List<Vector3>());
            wanted_positions.Add("ArmedCar (5)", new List<Vector3>());
            
            foreach(GameObject go in GameObject.FindObjectsOfType(typeof(GameObject)))
            {
                if(go.name == "Cube")
                    walls.Add(go);
            }
        }


        private void FixedUpdate()
        {
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Execute your path here
            
            GameObject lead_car = friends[0];
            Vector3 lead_position = lead_car.transform.position;
            float current_angle = lead_car.transform.rotation.eulerAngles.y;
            
            // Vector3 dir1 = new Vector3(35.0f, 0.0f, 10.0f);
            // dir1 = Quaternion.Euler(new Vector3(0.0f, current_angle, 0.0f)) * dir1;
            // dir1 = lead_position + dir1;
            // float dist1 = Math.Abs(lead_position.x - dir1.x) + Math.Abs(lead_position.z - dir1.z);
            // Vector3 dir2 = new Vector3(-35.0f, 0.0f, 10.0f);
            // dir2 = Quaternion.Euler(new Vector3(0.0f, current_angle, 0.0f)) * dir2;
            // dir2 = lead_position + dir2;
            // float dist2 = Math.Abs(lead_position.x - dir2.x) + Math.Abs(lead_position.z - dir2.z);
            
            // Debug.DrawLine(lead_position, dir1);
            // Debug.DrawLine(lead_position, dir2);
            
            // bool found_cube = false;
            //
            // RaycastHit[] all_hits = Physics.RaycastAll(lead_position, dir1, dist1);
            // foreach (var hit in all_hits)
            // {
            //     if (hit.transform.name == "Cube")
            //     {
            //         found_cube = true;
            //         break;
            //     }
            // }
            // all_hits = Physics.RaycastAll(lead_position, dir2, dist2);
            // foreach (var hit in all_hits)
            // {
            //     if (hit.transform.name == "Cube")
            //     {
            //         found_cube = true;
            //         break;
            //     }
            // }
            //
            // Dictionary<string, Vector3> current_relative_positions = relative_positions;

            // if (found_cube)
            // {
            //     current_relative_positions = new Dictionary<string, Vector3>();
            //     current_relative_positions.Add("ArmedCar (2)", new Vector3(-5.0f, 0.0f, -10.0f));
            //     current_relative_positions.Add("ArmedCar (3)", new Vector3(5.0f, 0.0f, -20.0f));
            //     current_relative_positions.Add("ArmedCar (4)", new Vector3(-5.0f, 0.0f, -30.0f));
            //     current_relative_positions.Add("ArmedCar (5)", new Vector3(5.0f, 0.0f, -40.0f));
            // }

            for (int f_i = 1; f_i < friends.Length; f_i++)
            {
                Vector3 relative_position = relative_positions[friends[f_i].name];
                relative_position = Quaternion.Euler(new Vector3(0.0f, current_angle, 0.0f)) * relative_position;
                Vector3 wanted_position = lead_position + relative_position;

                int mult = 1;
                if (relative_positions[friends[f_i].name].x > 0.0f)
                {
                    mult = -1;
                }

                Vector3 added_pos = new Vector3(4.0f, 0.0f, 0.0f);
                for (int f_k = 0; f_k < 40; f_k++)
                {
                    if (is_in_wall(wanted_position))
                    {
                        relative_position = relative_positions[friends[f_i].name] + mult * added_pos;
                        relative_position = Quaternion.Euler(new Vector3(0.0f, current_angle, 0.0f)) * relative_position;
                        wanted_position = lead_position + relative_position;
                        added_pos += new Vector3(4.0f, 0.0f, 0.0f);
                    }
                    else
                    {
                        break;
                    }
                }

                if (wanted_positions[friends[f_i].name].Count > 100){
					wanted_positions[friends[f_i].name].RemoveAt(0);
				}
				wanted_positions[friends[f_i].name].Add(wanted_position);
                
                for (int f_j = 0; f_j < wanted_positions[friends[f_i].name].Count; f_j++){
                    if (Math.Abs(friends[f_i].transform.position.x - wanted_positions[friends[f_i].name][f_j].x) +
                        Math.Abs(friends[f_i].transform.position.z - wanted_positions[friends[f_i].name][f_j].z) <
                        30.0f)
                    {
                        wanted_positions[friends[f_i].name].RemoveAt(0);
                    }
                }
                
                for (int f_j = 1; f_j < wanted_positions[friends[f_i].name].Count; f_j++){
                    Debug.DrawLine(wanted_positions[friends[f_i].name][f_j - 1], wanted_positions[friends[f_i].name][f_j]);
                }

                // Debug.DrawLine(wanted_position, wanted_position_check);
                //Debug.DrawLine(friends[f_i].transform.position, wanted_position);
                // Debug.DrawLine(lead_position, wanted_position);

                wanted_position = wanted_positions[friends[f_i].name][0];

                Vector3 direction = (wanted_position - friends[f_i].transform.position).normalized;

                float dist = Math.Abs(wanted_position.x - friends[f_i].transform.position.x) +
                             Math.Abs(wanted_position.z - friends[f_i].transform.position.z);

                bool is_to_the_right = Vector3.Dot(direction, friends[f_i].transform.right) > 0f;
                bool is_to_the_front = Vector3.Dot(direction, friends[f_i].transform.forward) > 0f;

                float steering = 0f;
                // float acceleration = 0f;
                float acceleration = Mathf.Min(1.0f, Mathf.Max(0.25f, ((Mathf.Min(25.0f, dist) - 10.0f) / 15.0f)));

                if (is_to_the_right && is_to_the_front)
                {
                    steering = 1f;
                    // acceleration = 1f;
                }
                else if (is_to_the_right && !is_to_the_front)
                {
                    steering = 1f;
                    acceleration = 0.75f;
                }
                else if (!is_to_the_right && is_to_the_front)
                {
                    steering = -1f;
                    // acceleration = 1f;
                }
                else if (!is_to_the_right && !is_to_the_front)
                {
                    steering = -1f;
                    acceleration = 0.75f;
                }

                // this is how you access information about the terrain
                int i = terrain_manager.myInfo.get_i_index(transform.position.x);
                int j = terrain_manager.myInfo.get_j_index(transform.position.z);
                float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
                float grid_center_z = terrain_manager.myInfo.get_z_pos(j);


                // this is how you control the car
                //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
                friends[f_i].GetComponent<CarController>().Move(steering, acceleration, acceleration, 0f);
                //m_Car.Move(0f, -1f, 1f, 0f);
            }
        }
    }
}
