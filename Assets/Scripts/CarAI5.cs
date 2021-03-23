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
        public bool DEBUG = true;                                                               // Enable/disable debugging.
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

        int rrf_resolution = 45;                                                                // Angle resolution in degrees of Rotating Rigid Formation Path Finding.       TODO: Update placeholder value.

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

            if (DEBUG && this_id == 0)
            {
                // ----- Print current target position in i,j grid -----
                //int target_pos_i = terrain_manager.myInfo.get_i_index(current_target_position[0]);
                //int target_pos_j = terrain_manager.myInfo.get_j_index(current_target_position[2]);
                //Debug.Log("Target grid position: i=" + target_pos_i + ", j=" + target_pos_j);
                // ----- /Print current target position in i,j grid -----


                // ----- Visualize the Rotated Rigid Formations -----
                float position_1_x = terrain_manager.myInfo.get_x_pos(5);                       // Coordinates (5,9).
                float position_1_z = terrain_manager.myInfo.get_z_pos(9);
                Vector3 position_1 = new Vector3(position_1_x, 0.0f, position_1_z);
                float position_2_x = terrain_manager.myInfo.get_x_pos(6);                       // Coordinates (6,9).
                float position_2_z = terrain_manager.myInfo.get_z_pos(9);
                Vector3 position_2 = new Vector3(position_2_x, 0.0f, position_2_z);
                float position_3_x = terrain_manager.myInfo.get_x_pos(7);                       // Coordinates (7,9).
                float position_3_z = terrain_manager.myInfo.get_z_pos(9);
                Vector3 position_3 = new Vector3(position_3_x, 0.0f, position_3_z);

                //Debug.Log(position_1);
                //Debug.Log(position_2);
                //Debug.Log(position_3);

                List<Vector3> formation_pattern = new List<Vector3>();
                formation_pattern.Add(position_1);
                formation_pattern.Add(position_2);
                formation_pattern.Add(position_3);

                List<RigidFormation> possible_rigid_formations = GetRotatedRigidFormations(formation_pattern, rrf_resolution);

                //Debug.Log("---------------------------------------------------");

                foreach (RigidFormation rf in possible_rigid_formations)
                {
                    //Debug.Log("--- Formation:");

                    Vector3 leader = rf.agent_positions[0];
                    foreach (Vector3 agent_pos in rf.agent_positions)
                    {
                        //Debug.Log(agent_pos);

                        //Color color = Color.magenta;
                        //Debug.DrawLine(leader, agent_pos, color, float.PositiveInfinity);
                    }
                }
                // ----- /Visualize the Rotated Rigid Formations -----
            }
        }

        private void FixedUpdate()
        {
            GameObject current_car = m_Car.gameObject;
            this_position = current_car.transform.position;                                     // Update current agent's position to current.


            if (DEBUG)
            {
                // ----- Log to console when current agent can see an enemy -----
                //foreach (GameObject enemy in enemies)
                //{
                //    Vector3 enemy_position = enemy.transform.position;
                //    if (CheckInSight(this_position, enemy_position))
                //    {
                //        Debug.Log("--- Enemy visible ---");
                //    }
                //}
                // ----- /Log to console when current agent can see an enemy -----
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

        public void RRFPathFinding(List<Vector3> formation_pattern, float[,] environment)
        {
            // Rotating Rigid Formation Path Finding.
            List<List<Vector3>> possible_formation_patterns = GetRotatedFormationPatterns(formation_pattern, rrf_resolution);
            bool move_successful;
            float x;
            float z;
            float current_rotation = 0.0f;                                                                                  // Indicates current rotation angle.
            float survivability;                                                                                            // Survivability score of current formation at current position.
            RFNode node;
            RFGraph rf_graph;
            List<RFGraph> rf_graph_list = new List<RFGraph>();
            int rf_node_nr;                                                                                                 // Unique number of each node for each rf graph.
            int num_vertices = weighted_positions.Length;                                                                   // Number of cells in the weighted grid.
            float formation_height;                                                                                         // Height of the current formation.
            float formation_width;                                                                                          // Width of the current formation.
            foreach (List<Vector3> formation_pattern in possible_formation_patterns)                                        // Iterate over each possible formation rotation.
            {
                rf_graph = new RFGraph(num_vertices, current_rotation);                                                     // Initialize a graph with same size as the weighted grid.
                rf_node_nr = 0;
                x = x_low;                                                                                                  // Set the starting position to top left.
                z = z_high;                                                                                                 // ...
                RigidFormation rf = new RigidFormation(formation_pattern);                                                  // Create a RigidFormation object of the formation pattern.
                formation_height = rf.GetFormationHeight();
                formation_width = rf.GetFormationWidth();
                for (int i = 0; i < z_high + formation_height; i++)                                                         // Iterate over map height + formation height.
                {
                    for (int j = 0; j < x_high + formation_width; j++)                                                      // Iterate over map width + formation width.
                    {
                        rf = MoveFormation(rf, new Vector3(x, 0.0f, z));                                                    // Move formation and create a new RigidFormation object.
                        if (rf is not null)                                                                                 // Check that the move was possible (i.e no agent hit an obstacle).
                        {
                            survivability = rf.GetSurvivability(weighted_positions);                                        // Get the formation survivability at current position.
                            node = new RFNode(rf_graph, x, z, current_rotation, survivability, rf_node_nr);                 // Create a node for current position and survivability.
                            // ################################# TODO: Keep RFNode or not? ###########################################################################
                            rf_graph.AddRigidFormation(rf);

                            // #################### FORTSÄTT HÄR! NU LÄGGS rf TILL I rf_graph ISTÄLLET FÖR RFNODES ##############################
                            // #################### ÄNDRA ConnectNeighbors SÅ ATT DEN TAR HÄNSYN TILL DET ISTÄLLET ##############################

                            // Link nodes in rf_graph with its adjacent vertices and set the weight of every outgoing edge to be the adjacent vertex's TSC.
                            ConnectNeighbors(rf_graph, node, current_rotation);                                             // Connect node with previously created neighbors in the rf_graph.
                            rf_node_nr++;
                        }
                        x += 1.0f;                                                                                          // Add one step to the right.
                    }
                    x = x_low;                                                                                              // Reset x to leftmost position.
                    z -= 1.0f;                                                                                              // Shift down one row.
                }
                current_rotation += rrf_resolution;                                                                         // Update to next rotation.
                rf_graph_list.Add(rf_graph);                                                                                // Add the current graph to list of graphs that will later be connected.
            }

            // Mutually link vertices from rf_graph[i] containing joint agents that are also placed on the same positions in vertices from rf_graph[i+1].
            for (int i = 0; i < rf_graph_list.Count; i++)                                                                   // Iterate over each rf_graph.
            {
                foreach (KeyValuePair<(float, float, float), RFNode> kvp in rf_graph_list[i].node_dictionary)
                {
                    (float, float, float) same_position = (kvp.Key.Item1, kvp.Key.Item2, rf_graph_list[i + 1].rotation);    // Same 
                    if (rf_graph_list[i + 1].node_dictionary.ContainsKey(kvp.Key))
                    {
                        rf_graph_list[i].AddEdge(blabla rf_graph_list[i + 1]);
                        rf_graph_list[i + 1].AddEdge(blabla rf_graph_list[i]);
                    }
                }
            }
        }

        public List<RigidFormation> GetRotatedFormationPatterns(List<Vector3> formation_pattern, int rotation_resolution)
        {
            // Returns all possible rotations of a formation pattern given an angle resolution of the rotations in degrees.
            List<List<Vector3>> rotated_formation_patterns = new List<List<Vector3>>();                                             // List where the rotated formation patterns will be saved.
            for (int angle = 0; angle < 360; angle += rotation_resolution)                                                          // Create all possible rotations of the formation pattern.
            {
                Vector3 pivot = formation_pattern[0];                                                                               // Pivot where the pattern will rotate around.
                for (int i = 0; i < formation_pattern.Count; i++)                                                                   // Loop over positions in the pattern.
                {
                    formation_pattern[i] = RotateAroundPoint(formation_pattern[i], pivot, Quaternion.Euler(0, angle, 0));           // Rotate with angle around the y-axis.
                }
                List<Vector3> new_formation_pattern = new List<Vector3>(formation_pattern);                                         // Clone the formation pattern so it is not changed next iteration.
                rotated_formation_patterns.Add(new_formation_pattern);                                                              // Add the formation pattern to the list.
            }
            return rotated_formation_patterns;
        }

        public Vector3 RotateAroundPoint(Vector3 point, Vector3 pivot, Quaternion angle) {
            // Rotates a point around another point as pivot.
            return angle * (point - pivot) + pivot;                                                                                 // TODO: Check how this handles rotations for different angles.
        }

        public RigidFormation MoveFormation(RigidFormation rf, Vector3 new_leader_position)
        {
            // Moves the formation. Returns new RigidFormation if successful and false if an agent ends up in an obstacle.
            Vector3 old_leader_position = rf.agent_positions[0];
            Vector3 position_difference = new_leader_position - old_leader_position;
            List<Vector3> new_agent_positions = new List<Vector3>(rf.agent_positions);
            for (int i = 0; i < rf.nr_of_agents; i++)
            {
                new_agent_positions[i] += position_difference;
                if (CheckObstacle(new_agent_positions[i][0], new_agent_positions[i][2]))
                {
                    return null;
                }
            }
            return new_agent_positions;
        }

        public void ConnectNeighbors(RFGraph rf_graph, RFNode node, float rotation)
        {
            // Above:   (node.x - 1, node.z - 1), (node.x, node.z - 1), (node.x + 1, node.z - 1);
            // Sides:   (node.x - 1, node.z), (node.x + 1, node.z);
            // Below:   (node.x - 1, node.z + 1), (node.x, node.z + 1), (node.x + 1, node.z + 1);
            (float x, float z)[] up_left_positions = new (float, float)[] { (node.x - 1, node.z - 1), (node.x, node.z - 1), (node.x + 1, node.z - 1), (node.x - 1, node.z) };
            foreach ((float, float) position in up_left_positions)
            {
                (float, float, float) key = (position.Item1, position.Item2, rotation);                                             // Position including rotation as key in dictionary.
                if (rf_graph.node_dictionary.TryGetValue(key, out RFNode adj_node))                                                 // Checks if there is a neighbor at this position.
                {
                    // "node" and "adj_node" are neighbours.
                    // Add edges between them with the opposite node's survivability score as edge weight.
                    rf_graph.AddEdge(node.rf_node_nr, adj_node.rf_node_nr, adj_node.survivability);
                    rf_graph.AddEdge(adj_node.rf_node_nr, node.rf_node_nr, node.survivability);
                }
            }
        }

        public void OnDrawGizmos()
        {
            // Debugging in Scene view.
            if (DEBUG && this_id == 0)
            {
                Handles.Label(this_position, "ID: " + this_id);                                                                     // Show ID of the cars.

                Gizmos.DrawSphere(new Vector3(x_low, 10.0f, z_high), 5);                                                            // Visualize a point.

                // ----- Create and visualize Rotated Rigid Formations -----
                //float position_1_x = terrain_manager.myInfo.get_x_pos(5);                       // Coordinates (5,9).
                //float position_1_z = terrain_manager.myInfo.get_z_pos(9);
                //Vector3 position_1 = new Vector3(position_1_x, 0.0f, position_1_z);
                //float position_2_x = terrain_manager.myInfo.get_x_pos(6);                       // Coordinates (6,9).
                //float position_2_z = terrain_manager.myInfo.get_z_pos(9);
                //Vector3 position_2 = new Vector3(position_2_x, 0.0f, position_2_z);
                //float position_3_x = terrain_manager.myInfo.get_x_pos(7);                       // Coordinates (7,9).
                //float position_3_z = terrain_manager.myInfo.get_z_pos(9);
                //Vector3 position_3 = new Vector3(position_3_x, 0.0f, position_3_z);

                //List<Vector3> formation_pattern = new List<Vector3>();
                //formation_pattern.Add(position_1);
                //formation_pattern.Add(position_2);
                //formation_pattern.Add(position_3);

                //Gizmos.color = Color.blue;
                //GUIStyle style_original = new GUIStyle();
                //style_original.normal.textColor = Color.blue;
                //Gizmos.DrawSphere(position_1, 1);
                //Handles.Label(position_1 + Vector3.back * 3 + Vector3.left * 6, "" + position_1, style_original);
                //Gizmos.DrawSphere(position_2, 1);
                //Handles.Label(position_2 + Vector3.back * 3 + Vector3.left * 6, "" + position_2, style_original);
                //Gizmos.DrawSphere(position_3, 1);
                //Handles.Label(position_3 + Vector3.back * 3 + Vector3.left * 6, "" + position_3, style_original);

                //List<RigidFormation> possible_rigid_formations = GetRotatedRigidFormations(formation_pattern, rrf_resolution);

                //foreach (RigidFormation rf in possible_rigid_formations)
                //{
                //    Vector3 leader = rf.agent_positions[0];
                //    int f_index = 0;
                //    foreach (Vector3 agent_pos in rf.agent_positions)
                //    {
                //        Gizmos.color = Color.magenta;
                //        Gizmos.DrawSphere(agent_pos, 1);
                //        Handles.Label(agent_pos + Vector3.back + Vector3.left * 6, "" + f_index + ". " + agent_pos);
                //        f_index++;
                //    }
                //}
                // ----- /Visualize the Rotated Rigid Formations -----


                // ----- Draw scores in a grid fashion -----
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
                // ----- /Draw scores in a grid fashion -----
            }
        }
}


    public class RigidFormation
    {
        public int nr_of_agents;
        public List<Vector3> agent_positions;

        public RigidFormation(List<Vector3> agent_positions)
        {
            // Input: Positions of the agents sorted, with the first one being the leader.
            this.nr_of_agents = agent_positions.Count;
            this.agent_positions = agent_positions;
            agent_positions.Sort((x, y) => {                                                                    // Sort by x-position first and then by y-position.     TODO: Check if it works.
                var ret = x[0].CompareTo(y[0]);                                                                 // TODO: Is sorting necessary?
                if (ret == 0) ret = x[2].CompareTo(y[2]);
                return ret;
            });
        }

        public Vector3 GetLeaderPosition()
        {
            return this.agent_positions[0];
        }

        public float GetFormationWidth()
        {
            // Returns the horisontal size of the formation.
            float x_min = float.PositiveInfinity;
            float x_max = float.NegativeInfinity;
            foreach (Vector3 agent_pos in this.agent_positions)
            {
                x_min = Math.Min(x_min, agent_pos[0]);
                x_max = Math.Max(x_max, agent_pos[0]);
            }
            return x_max - x_min;
        }

        public float GetFormationHeight()
        {
            // Returns the vertical size of the formation.
            float z_min = float.PositiveInfinity;
            float z_max = float.NegativeInfinity;
            foreach (Vector3 agent_pos in this.agent_positions)
            {
                z_min = Math.Min(z_min, agent_pos[2]);
                z_max = Math.Max(z_max, agent_pos[2]);
            }
            return z_max - z_min;
        }

        public float GetSurvivability(float[,] weighted_positions)
        {
            // Returns the survivability of the formation at their current positions.
            // Input: Grid with survivability score at each position.
            float formation_survivability = 0;
            foreach (Vector3 agent_pos in this.agent_positions)
            {
                formation_survivability += weighted_positions[(int)agent_pos[0], (int)agent_pos[2]];        // TODO: Change from addition to something else (?).
            }
            return formation_survivability;
        }

        public override bool Equals(Object obj)
        {
            // Compare two rigid formations if they have all agents at the exact same position (including order).               TODO: Should order be taken into account or not?
            if ((obj == null) || !this.GetType().Equals(obj.GetType()))                                     // Check for null and compare run-time types.
            {
                return false;
            }
            else
            {
                RigidFormation other_rf = (RigidFormation)obj;
                for (int i = 0; i < this.nr_of_agents; i++)
                {
                    if (!this.agent_positions[i] == other_rf.agent_positions[i])
                    {
                        return false
                    }
                }
                // Check for equality without taking order into account.
                //foreach (Vector3 pos in this.agent_positions)
                //{
                //    if (!other_rf.agent_positions.Contains(pos))                                            // Check if the position exists in the other rigid formation.
                //    {
                //        return false
                //    }
                //}
                return true;
            }
        }
    }


    public class RFNode
    {
        //public static Dictionary<(float, float, float), RFNode> node_dictionary = new Dictionary<(float, float, float), RFNode>();     // Dictionary of all nodes with (x, z, rotation) tuple as key.
        private static float cell_size = 1.0f;                                              // Size of the cells in the grid. Should correspond to the grid resolution.
        public float x;
        public float z;
        public float rotation;
        public float survivability;                                                         // Measure of team survivability at this position.
        public int rf_node_nr;                                                        
        private List<RFNode> neighbors = new List<RFNode>();                                // List of adjacent non-obstacle nodes.

        public RFNode(RFGraph rf_graph, float x, float z, float rotation, float survivability, int rf_node_nr)
        {
            this.x = x;
            this.z = z;
            this.rotation = rotation;
            this.survivability = survivability;
            this.rf_node_nr = rf_node_nr;
            rf_graph.AddNode((this.x, this.z), this);                                       // Add this node to a dictionary in the graph.
        }

        public bool CheckAdjacent(RFNode node)
        {
            // Checks if this node is adjacent to or on the same position as the input node.
            if (node.x == this.x || Math.Abs(node.x - this.x) == cell_size)                 // Same or adjacent x position.
            {
                if (node.z == this.z || Math.Abs(node.z - this.z) == cell_size)             // Same or adjacent z position.
                {
                    return true;
                }
            }
            return false;
        }

        public void AddNeighbor(RFNode adjacent_node)
        {
            this.neighbors.Add(adjacent_node);
        }

        //public static RFNode GetNode(float x, float z, float rotation)                // Moved to RFGraph.
        //{
        //    // Returns the node object given x and z coordinates and rotation.
        //    return node_dictionary[(x, z, rotation)];
        //}
    }


    public class RFGraph
    {
        public Dictionary<(float, float, float), RFNode> node_dictionary = new Dictionary<(float, float, float), RFNode>();     // Dictionary of all nodes with (x, z, rotation) tuple as key.
        public List<RigidFormation> rigid_formations;
        private int num_vertices;
        private float[,] adj_matrix;
        public float rotation;

        public RFGraph(int num_vertices, float rotation)
        {
            // Initialize the matrix.
            this.num_vertices = num_vertices;
            this.adj_matrix = new float[num_vertices, num_vertices];
            this.rotation = rotation;
            this.rigid_formations = new List<RigidFormation>();
        }

        public void AddEdge(int node_1, int node_2, float weight)
        {
            // Add an edge from node 1 to node 2.
            if (node_1 >= this.num_vertices || node_2 >= this.num_vertices || node_1 < 0 || node_2 < 0)
            {
                throw new ArgumentOutOfRangeException("Vertices out of bounds.");
            }
            this.adj_matrix[node_1, node_2] = weight;
        }

        public void AddNode(float x, float z, RFNode node)
        {
            this.node_dictionary.Add((x, z, rotation), node);
        }

        public void AddRigidFormation(RigidFormation rf)
        {
            this.rigid_formations.Add(rf);
        }

        //public void GetAdjacent(int node)
        //{
        //    for (int i = 0; i < num_vertices; i++)
        //    {

        //    }
        //}
    }
}
