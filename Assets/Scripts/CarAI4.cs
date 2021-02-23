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

        public Agent this_agent;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");                  // Contains self, replay car & friends.
            enemies = GameObject.FindGameObjectsWithTag("Enemy");


            // Plan your path here
            // ...

            // ---------- Assigns an ID to each car from their starting positions. ----------
            Vector3 friend_pos;
            Vector3 this_pos = gameObject.transform.position;
            int this_id = 0;
            int i = 0;
            foreach (GameObject friend in friends)
            {
                i++;
                if (i == 1)                                                         // Skip first agent in friends since that is the replay car.
                {
                    continue;
                }
                friend_pos = friend.transform.position;
                if (this_pos[0] > friend_pos[0])                                      // If this agent is to the right of friend.
                {
                    this_id++;
                }
            }
            this_agent = new Agent(this_id);
            // ---------- /Assigns an ID to each car from their starting positions. ----------
        }


        private void FixedUpdate()
        {
            // Execute your path here
            // ...

            // Just testing if the ID works.
            if (this_agent.id == 0)
            {
                //Debug.Log("I am car 1 and will turn left.");
                m_Car.Move(-1f, 1f, 1f, 0f);
            }
            else if (this_agent.id == 1)
            {
                //Debug.Log("I am car 2 and will turn slight left.");
                m_Car.Move(-0.1f, 1f, 1f, 0f);
            }
            else if (this_agent.id == 2)
            {
                //Debug.Log("I am car 3 and will turn slight right.");
                m_Car.Move(0.1f, 1f, 1f, 0f);
            }
            else if (this_agent.id == 3)
            {
                //Debug.Log("I am car 4 and will turn right.");
                m_Car.Move(1f, 1f, 1f, 0f);
            }


            //Vector3 direction = (avg_pos - transform.position).normalized;

            //bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            //bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            //float steering = 0f;
            //float acceleration = 0;

            //if (is_to_the_right && is_to_the_front)
            //{
            //    steering = 1f;
            //    acceleration = 1f;
            //}
            //else if (is_to_the_right && !is_to_the_front)
            //{
            //    steering = -1f;
            //    acceleration = -1f;
            //}
            //else if (!is_to_the_right && is_to_the_front)
            //{
            //    steering = -1f;
            //    acceleration = 1f;
            //}
            //else if (!is_to_the_right && !is_to_the_front)
            //{
            //    steering = 1f;
            //    acceleration = -1f;
            //}

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
    }


    public class Agent
    {
        public int id;                                         // Each agent is numbered with an unique ID.
        public Vector3 position;                               // Current position of the agent.

        public Agent(int id)
        {
            this.id = id;
        }
    }
}
