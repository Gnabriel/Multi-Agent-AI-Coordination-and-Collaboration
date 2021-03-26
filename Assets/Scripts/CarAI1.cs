using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI1 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

        private Dictionary<Vector3Int, HashSet<Vector3Int>> adjacencies;
        private Dictionary<Vector3Int, HashSet<Vector3Int>> mst;
        private Dictionary<Vector3Int, HashSet<Vector3Int>> mst_only;
        private HashSet<Vector3Int> mst_nodes;
        private HashSet<Vector3Int> mst_nodes_only;
        private Vector3Int[] current_nodes;
        private float INF = 999999999.0f;

        public Vector3 get_pos(Vector3Int pos)
        {
            float step_x = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float x = terrain_manager.myInfo.x_low + step_x / 2 + step_x * pos.x;
            float step_z = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            float z = terrain_manager.myInfo.z_low + step_z / 2 + step_z * pos.z;
            return new Vector3(x, 0.0f, z);
        }

        public Vector3Int get_closest(Vector3Int current_pos, Vector3 pos, int car_id)
        {
            if (current_pos != new Vector3Int(-1, 0, -1))
            {
                foreach (Vector3Int other_pos in mst_only[current_pos])
                {
                    if (mst_nodes.Contains(other_pos))
                    {
                        return other_pos;
                    }
                }
                
            }
            
            HashSet<Vector3Int> nodes_to_visit_by_other_cars = new HashSet<Vector3Int>();
            for (int i = 0; i < current_nodes.Length; i++)
            {
                if (i != car_id)
                {
                    Vector3Int current_nod = current_nodes[i];
                    while (current_nod != new Vector3Int(-1, 0, -1) && !nodes_to_visit_by_other_cars.Contains(current_nod))
                    {
                        nodes_to_visit_by_other_cars.Add(current_nod);
                        if (mst.ContainsKey(current_nod))
                        {
                            foreach (Vector3Int new_nod in mst[current_nod])
                            {
                                current_nod = new_nod;
                                break;
                            }
                        }
                        else
                        {
                            current_nod = new Vector3Int(-1, 0, -1);
                        }
                    }
                }
            }

            float current_dist = INF;
            Vector3Int current_node = new Vector3Int((int) pos.x, 0, (int) pos.z);
            foreach (Vector3Int node in mst_nodes_only)
            {
                Vector3 node_pos = get_pos(node);
                float new_dist = Math.Abs(pos.x - node_pos.x) + Math.Abs(pos.z - node_pos.z);
                if (new_dist < current_dist && node != current_pos && !nodes_to_visit_by_other_cars.Contains(node))
                {
                    current_dist = new_dist;
                    current_node = node;
                }
            }
            
            if (current_pos != new Vector3Int(-1, 0, -1)){
                return dijkstra(current_pos, current_node);
            }

            return current_node;
        }

        private Vector3Int dijkstra(Vector3Int current_pos, Vector3Int goal_pos)
        {
            Dictionary<Vector3Int, float> dists = new Dictionary<Vector3Int, float>();
            Dictionary<Vector3Int, Vector3Int> prevs = new Dictionary<Vector3Int, Vector3Int>();
            HashSet<Vector3Int> nodes_to_visit = new HashSet<Vector3Int>();

            foreach (Vector3Int node in mst_nodes)
            {
                dists[node] = INF;
                prevs[node] = new Vector3Int(-1, 0, -1);
                nodes_to_visit.Add(node);
            }

            dists[current_pos] = 0.0f;
            while (nodes_to_visit.Count != 0)
            {
                Vector3Int current = new Vector3Int(-1, 0, -1);
                float current_dist = INF;
                foreach (Vector3Int node in nodes_to_visit)
                {
                    if (dists[node] < current_dist)
                    {
                        current_dist = dists[node];
                        current = node;
                    }
                }

                nodes_to_visit.Remove(current);

                foreach (Vector3Int node in adjacencies[current])
                {
                    float dist_alt = current_dist + Math.Abs(node.x - current.x) + Math.Abs(node.z - current.z);
                    if (dist_alt < dists[node])
                    {
                        dists[node] = dist_alt;
                        prevs[node] = current;
                    }
                }
            }

            Vector3Int new_current = goal_pos;
            while (new_current != new Vector3Int(-1, 0, -1))
            {
                if (prevs[new_current] == current_pos)
                {
                    break;
                }
                new_current = prevs[new_current];
            }
            
            return new_current;
        }

        private bool check_cycles(Dictionary<Vector3Int, HashSet<Vector3Int>> graph, HashSet<Vector3Int> visited, Vector3Int current, Vector3Int adj)
        {
            if (current == adj)
            {
                return true;
            }

            if (!visited.Contains(adj))
            {
                visited.Add(adj);
                if (graph.ContainsKey(adj))
                {
                    foreach (Vector3Int adj2 in graph[adj])
                    {
                        if (check_cycles(graph, visited, current, adj2))
                        {
                            return true;
                        }
                    }
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

           


            // Plan your path here
            adjacencies = new Dictionary<Vector3Int, HashSet<Vector3Int>>();
            if (m_Car.name == friends[0].name)
            {
                for (int i = 0; i < terrain_manager.myInfo.x_N; i++)
                {
                    for (int j = 0; j < terrain_manager.myInfo.z_N; j++)
                    {
                        if (terrain_manager.myInfo.traversability[i, j] < 0.5f)
                        {
                            Vector3Int current_pos = new Vector3Int(i, 0, j);
                            adjacencies.Add(current_pos, new HashSet<Vector3Int>());
                            foreach (Vector3Int pos in adjacencies.Keys)
                            {
                                float dist = Math.Abs(pos.x - current_pos.x) + Math.Abs(pos.z - current_pos.z);
                                if (dist == 1)
                                {
                                    adjacencies[current_pos].Add(pos);
                                    adjacencies[pos].Add(current_pos);
                                }
                            }
                        }
                    }
                }

                mst = new Dictionary<Vector3Int, HashSet<Vector3Int>>();
                mst_only = new Dictionary<Vector3Int, HashSet<Vector3Int>>();
                mst_nodes = new HashSet<Vector3Int>();
                mst_nodes_only = new HashSet<Vector3Int>();
                HashSet<Tuple<Vector3Int, Vector3Int>> edges = new HashSet<Tuple<Vector3Int, Vector3Int>>();
                foreach (Vector3Int pos in adjacencies.Keys)
                {
                    foreach (Vector3Int adj in adjacencies[pos])
                    {
                        if (!edges.Contains(new Tuple<Vector3Int, Vector3Int>(adj, pos)))
                        {
                            edges.Add(new Tuple<Vector3Int, Vector3Int>(pos, adj));
                        }
                    }
                }
                HashSet<Vector3Int> visited_positions = new HashSet<Vector3Int>();
                HashSet<Tuple<Vector3Int, Vector3Int>> visited_edges = new HashSet<Tuple<Vector3Int, Vector3Int>>();
                foreach (Tuple<Vector3Int,Vector3Int> edge in edges)
                {
                    if (visited_edges.Count != adjacencies.Keys.Count - 1)
                    {
                        if (!check_cycles(mst, new HashSet<Vector3Int>(), edge.Item2, edge.Item1))
                        {
                            if (!mst.ContainsKey(edge.Item1))
                            {
                                mst.Add(edge.Item1, new HashSet<Vector3Int>());
                                mst_only.Add(edge.Item1, new HashSet<Vector3Int>());
                                mst_nodes.Add(edge.Item1);
                                mst_nodes_only.Add(edge.Item1);
                            }

                            mst[edge.Item1].Add(edge.Item2);
                            mst_only[edge.Item1].Add(edge.Item2);
                            visited_positions.Add(edge.Item1);

                            if (!mst.ContainsKey(edge.Item2))
                            {
                                mst.Add(edge.Item2, new HashSet<Vector3Int>());
                                mst_only.Add(edge.Item2, new HashSet<Vector3Int>());
                                mst_nodes.Add(edge.Item2);
                                mst_nodes_only.Add(edge.Item2);
                            }
                            
                            mst[edge.Item2].Add(edge.Item1);
                            mst_only[edge.Item2].Add(edge.Item1);
                            visited_positions.Add(edge.Item2);
                            
                            visited_edges.Add(edge);
                        }
                    }
                }
                
                current_nodes = new Vector3Int[friends.Length];
                for (int i = 0; i < friends.Length; i++)
                {
                    Vector3Int closest_node = get_closest(new Vector3Int(-1, 0, -1), friends[i].transform.position, i);
                    mst_nodes_only.Remove(closest_node);
                    current_nodes[i] = closest_node;
                }
            }
        }


        private void FixedUpdate()
        {

            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Execute your path here

            for (int f_i = 0; f_i < friends.Length; f_i++)
            {
                Vector3Int current_wanted_pos = current_nodes[f_i];
                Vector3 wanted_pos = get_pos(current_wanted_pos);

                if (Math.Abs(friends[f_i].transform.position.x - wanted_pos.x) + Math.Abs(friends[f_i].transform.position.z - wanted_pos.z) <
                    7.5f)
                {
                    Vector3Int new_wanted_pos = get_closest(current_wanted_pos, friends[f_i].transform.position, f_i);
                    current_nodes[f_i] = new_wanted_pos;
                    wanted_pos = get_pos(new_wanted_pos);
                    mst_nodes_only.Remove(new_wanted_pos);
                    mst_only[current_wanted_pos].Remove(new_wanted_pos);
                    mst_only[new_wanted_pos].Remove(current_wanted_pos);
                }

                Debug.DrawLine(friends[f_i].transform.position, wanted_pos);

                Vector3 direction = (wanted_pos - friends[f_i].transform.position).normalized;

                bool is_to_the_right = Vector3.Dot(direction, friends[f_i].transform.right) > 0f;
                bool is_to_the_front = Vector3.Dot(direction, friends[f_i].transform.forward) > 0f;

                float steering = 0f;
                float acceleration = 0;

                if (is_to_the_right && is_to_the_front)
                {
                    steering = 1f;
                    acceleration = 3f;
                }
                else if (is_to_the_right && !is_to_the_front)
                {
                    steering = -1f;
                    acceleration = -3f;
                }
                else if (!is_to_the_right && is_to_the_front)
                {
                    steering = -1f;
                    acceleration = 3f;
                }
                else if (!is_to_the_right && !is_to_the_front)
                {
                    steering = 1f;
                    acceleration = -3f;
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

            foreach (Vector3Int pos in mst_only.Keys)
            {
                foreach (Vector3Int pos2 in mst_only[pos])
                {
                    Debug.DrawLine(get_pos(pos), get_pos(pos2));
                }
            }

        }
    }
}
