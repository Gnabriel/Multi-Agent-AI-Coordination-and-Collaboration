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
        public List<Agent> agents;
        public float x;
        public float z;
        public int current_agent;
        public float max_dist;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Distances
            x = 5.0f;
            z = 5.0f;

            agents = new List<Agent>();
            current_agent = 0;

            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            for (int i = 1; i < friends.Length; i++)
            {
                float current_x = 0.0f;
                float current_z = 0.0f;
                if (i >= 3)
                {
                    current_x = agents[i - 2 - 1].wanted_x;
                    current_z = -agents[i - 2 - 1].wanted_z;
                }
                current_x += (2 * (i % 2) - 1) * x;
                current_z -= z;
                Agent agent = new Agent(current_x, current_z);
                agents.Add(agent);
                if (friends[i].name == m_Car.gameObject.name)
                {
                    current_agent = i - 1;
                }
            }

            // Calculate length of map diagonal.
            Vector3 top_left = new Vector3(terrain_manager.myInfo.x_low, 0.0f, terrain_manager.myInfo.z_low);
            Vector3 bottom_right = new Vector3(terrain_manager.myInfo.x_high, 0.0f, terrain_manager.myInfo.z_high);
            max_dist = Vector3.Distance(top_left, bottom_right);
        }


        private void FixedUpdate()
        {
            // Execute your path here
            GameObject lead_car = friends[0];
            GameObject current_car = m_Car.gameObject;
            Vector3 current_position = current_car.transform.position;
            Vector3 lead_position = lead_car.transform.position;

            Agent agent = agents[current_agent];
            float current_angle = lead_car.transform.rotation.y;
            float rotated_x = agent.wanted_x * Mathf.Cos(current_angle) + agent.wanted_z * Mathf.Sin(current_angle);
            float rotated_z = agent.wanted_z * Mathf.Cos(current_angle) - agent.wanted_x * Mathf.Sin(current_angle);
            Vector3 wanted_position = new Vector3(lead_position.x + rotated_x, lead_position.y, lead_position.z + rotated_z);

            Debug.DrawLine(lead_position, wanted_position, Color.red);

            Vector3 direction = (wanted_position - current_position).normalized;
            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            float steering = 0f;
            float acceleration = 0;

            Vector3 target_position = agent.GetTargetPosition(lead_position);

            Debug.Log("Acceleration: " + agent.GetAcceleration(max_dist, current_position, target_position));
            Debug.Log("Steering: " + agent.GetSteering(current_position, target_position));

            if (is_to_the_right && is_to_the_front)
            {
                steering = 0.5f;
                //steering = agent.GetSteering(current_position, target_position);
                acceleration = agent.GetAcceleration(max_dist, current_position, target_position);
            }
            else if (is_to_the_right && !is_to_the_front)
            {
                steering = -0.5f;
                //steering = -agent.GetSteering(current_position, target_position);
                acceleration = -agent.GetAcceleration(max_dist, current_position, target_position);
            }
            else if (!is_to_the_right && is_to_the_front)
            {
                steering = -0.5f;
                //steering = -agent.GetSteering(current_position, target_position);
                acceleration = agent.GetAcceleration(max_dist, current_position, target_position);
            }
            else if (!is_to_the_right && !is_to_the_front)
            {
                steering = 0.5f;
                //steering = agent.GetSteering(current_position, target_position);
                acceleration = -agent.GetAcceleration(max_dist, current_position, target_position);
            }

            m_Car.Move(steering, acceleration, acceleration, 0f);

            /*Vector3 avg_pos = Vector3.zero;

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
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            //m_Car.Move(0f, -1f, 1f, 0f);*/
        }
    }

    public class Agent
    {
        public float wanted_x;
        public float wanted_z;

        public Agent(float w_x, float w_z)
        {
            this.wanted_x = w_x;
            this.wanted_z = w_z;
        }

        public Vector3 GetTargetPosition(Vector3 lead_position)
        {
            return new Vector3(lead_position[0] + wanted_x, 0.0f, lead_position[2] + wanted_z);
        }

        public float GetAcceleration(float max_dist, Vector3 current_position, Vector3 target_position)
        {
            // Linear acceleration based on distance to target position.
            float speed_factor = 10.0f;
            float min_acceleration = 0.1f;

            float travel_dist = Vector3.Distance(current_position, target_position);
            float acceleration = Math.Max(min_acceleration, speed_factor * (travel_dist / max_dist));
            return acceleration;
        }

        public float GetSteering(Vector3 current_position, Vector3 target_position)
        {
            // Linear steering based on angle towards target position.
            Debug.DrawLine(current_position, target_position, Color.blue);

            float steer_factor = 10.0f;
            float min_steering = 0.05f;

            float steering = steer_factor * (Vector3.Angle(current_position, target_position) / 180);

            if (steering < min_steering)                                // If steering angle is less than min_steering we drive straight.
            {
                steering = 0.0f;
            }
            return steering;
        }
    }
}