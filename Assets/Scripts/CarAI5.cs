using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEditor;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI5 : MonoBehaviour
    {
        // ----- Parameters -----
        public float target_in_sight_score = 0.0f;
        public float not_exposed_score = 0.0f;
        // ----- /Parameters -----


        // ----- Initializations -----
        public Vector3 this_position;                                                           // The current agent's position (ie. the agent that is running the script).
        public int this_id;                                                                     // The current agent's ID (i.e index in friends list). Is assigned later.
        float[,] weighted_positions;                                                            // Matrix containing scores for all positions on the map (only whole integer coordinates).
        float x_low;                                                                            // Coordinates of map corners.
        float x_high;                                                                           // ...
        float z_low;                                                                            // ...
        float z_high;                                                                           // ...
        bool DEBUG_ONCE = true;

        private CarController m_Car;                                                            // The car controller that we want to use.
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public GameObject[] friends;
        public GameObject[] enemies;
        // ----- /Initializations -----


        private void Start()
        {
            
            m_Car = GetComponent<CarController>();                                              // Get the car controller
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            foreach (GameObject obj in enemies)
            {
                Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
            }


            // Plan your path here


            // ----- Initializations -----
            x_low = terrain_manager.myInfo.x_low;
            x_high = terrain_manager.myInfo.x_high;
            z_low = terrain_manager.myInfo.z_low;
            z_high = terrain_manager.myInfo.z_high;

            this_position = gameObject.transform.position;                                      // Starting position.
            this_id = -1;                                                                       // Temporary ID.

            GameObject current_target = GetTarget();                                            // Get the current target, which is the next target the agents will try to destroy.
            Vector3 current_target_position = current_target.transform.position;                // Position of current target.
            // ----- /Initializations -----


            int target_pos_i = terrain_manager.myInfo.get_i_index(current_target_position[0]);
            int target_pos_j = terrain_manager.myInfo.get_j_index(current_target_position[2]);
            Debug.Log("Target grid position: i=" + target_pos_i + ", j=" + target_pos_j);


            // ----- Assign an ID to the current car -----
            Vector3 friend_position;
            int i = 0;
            foreach (GameObject friend in friends)
            {
                friend_position = friend.transform.position;
                if (this_position.Equals(friend_position))                                      // If this agent is to the right of friend.
                {
                    this_id = i;
                }
                i++;
            }
            // ----- /Assign an ID to the current car -----


            // ----- Build weighted position matrix -----
            if (this_id == 0)                                                                   // Only build the matrix once (for the first agent in friends list).
            {
                weighted_positions = new float[(int)(x_high - x_low), (int)(z_high - z_low)];
                for (int x = (int)x_low; x < x_high; x++)
                {
                    for (int z = (int)z_low; z < z_high; z++)
                    {
                        float score = 0.0f;                                                     // Base score is zero.

                        if (CheckObstacle(x, z) is false)                                       // Only check obstacle-free positions.
                        {
                            // ----- "Target in sight"-score -----
                            Vector3 position = new Vector3(x, 0.0f, z);

                            if (CheckInSight(position, current_target_position))                // If the current target is in sight.
                            {
                                //score += target_in_sight_score;                                 // Add a score if the target is in sight.         ##### Weird fucking bug when using this #####
                                score += 50;                                                    // Add a score if the target is in sight.
                            }
                            // ----- /"Target in sight"-score -----


                            // ----- "Not exposed to enemies"-score -----
                            foreach (GameObject enemy in enemies)
                            {
                                Vector3 enemy_position = enemy.transform.position;
                                if (CheckInSight(position, enemy_position) is false)            // If not in sight of enemy.
                                {
                                    //score += not_exposed_score;                                 // Add a score if not exposed to enemy.           ##### Weird fucking bug when using this #####
                                    score += 10;                                                // Add a score if not exposed to enemy.
                                }
                            }
                            // ----- /"Not exposed to enemies"-score -----
                        }
                        int x_index = x - (int)x_low;                                           // Change from actual coordinates to indices in the matrix.
                        int z_index = z - (int)z_low;

                        //Debug.Log("Score at x:" + x + ", z:" + z + " = " + score);

                        weighted_positions[x_index, z_index] = score;                           // Save the score at this position in the matrix.
                    }
                }
            }
            else
            {
                weighted_positions = friends[0].GetComponent<CarAI5>().weighted_positions;
            }
            // ----- /Build weighted position matrix -----
        }


        private void FixedUpdate()
        {
            

            GameObject current_car = m_Car.gameObject;
            this_position = current_car.transform.position;                                     // Update position to current.

            foreach (GameObject enemy in enemies)
            {
                Vector3 enemy_position = enemy.transform.position;
                if (CheckInSight(this_position, enemy_position))
                {
                    Debug.Log("--- Enemy visible ---");
                }
            }
            

            // Execute your path here
            // ...

            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / friends.Length;
            Vector3 direction = (avg_pos - transform.position).normalized;

            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            float steering = 0f;
            float acceleration = 0;

            if (is_to_the_right && is_to_the_front)
            {
                steering = 1f;
                acceleration = 1f;
            }
            else if (is_to_the_right && !is_to_the_front)
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right && is_to_the_front)
            {
                steering = -1f;
                acceleration = 1f;
            }
            else if (!is_to_the_right && !is_to_the_front)
            {
                steering = 1f;
                acceleration = -1f;
            }

            // this is how you access information about the terrain
            //int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            //int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            //float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            //float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            //Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            //m_Car.Move(steering, acceleration, acceleration, 0f);
            //m_Car.Move(0f, -1f, 1f, 0f);
        }


        public bool CheckObstacle(float x, float z)
        {
            // Returns true if there is an obstacle at given position, otherwise false.
            int i = terrain_manager.myInfo.get_i_index(x);
            int j = terrain_manager.myInfo.get_j_index(z);
            if (terrain_manager.myInfo.traversability[i, j] == 1.0f)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        public bool CheckInSight(Vector3 position_1, Vector3 position_2)
        {
            // Returns true if the straight line between position 1 and 2 is collision-free.
            float distance = (position_1 - position_2).magnitude;
            Vector3 direction = (position_1 - position_2).normalized;
            RaycastHit hit;
            // Does the ray intersect any objects excluding the player layer
            int layer_mask = LayerMask.GetMask("CubeWalls");
            if (Physics.Raycast(position_2 + direction, direction, out hit, distance - 1f, layer_mask) is false)
            {
                return true;
            }
            else
            {
                return false;
            }
        }


        public GameObject GetTarget()
        {
            // Return the currently best target.
            return enemies[0];
        }


        public void RRFPathFinding(int formation_pattern, float[,] environment)
        {
            // Rotating Rigid Formation Path Finding.
        }


        public void OnGUI()
        {
            // Debugging in Game view.
            //Handles.Label(this_position, "ID: " + this_id);
        }


        void OnDrawGizmos()
        {
            // Debugging in Scene view.
            Handles.Label(this_position, "ID: " + this_id);

            // ----- Draw scores in a grid fashion -----
            if (this_id == 0)
            {
                int x_N = terrain_manager.myInfo.x_N;
                int z_N = terrain_manager.myInfo.z_N;
                Vector3 position;
                float score;
                float grid_center_x;
                float grid_center_z;
                GUIStyle style = new GUIStyle();
                style.normal.textColor = Color.green;
                for (int i = 0; i < x_N; i++)
                {
                    for (int j = 0; j < z_N; j++)
                    {
                        grid_center_x = terrain_manager.myInfo.get_x_pos(i);
                        grid_center_z = terrain_manager.myInfo.get_z_pos(j);
                        position = new Vector3(grid_center_x, 0.0f, grid_center_z);
                        score = weighted_positions[(int)Math.Floor(grid_center_x - x_low), (int)Math.Floor(grid_center_z - z_low)];
                        Handles.Label(position, "" + score, style);
                        //Handles.Label(position, "" + i + "," + j, style);
                    }
                }
            }
            // ----- /Draw scores in a grid fashion -----
        }
    }
}
