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
        public bool DEBUG = false;                                                               // Enable/disable debugging.
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
        int grid_resolution = 10;

        int rrf_resolution = 45;                                                                // Angle resolution in degrees of Rotating Rigid Formation Path Finding.       TODO: Update placeholder value.
        int current_rf_number;                                                                  // Which node in the multigraph we are at currently.
        Dictionary<int, RigidFormation> multi_rf_dict;                                          // Dictionary of all the rigid formation nodes in the final multigraph.

        static RFGraph traversal_graph;                                                         // Final multigraph of the Rotating Rigid Formation Path Finding.
        static bool target_rf_reached;                                                          // States if the target formation position is reached or not.
        static int target_rf_number;                                                            // The unique number of the target rigid formation.
        Vector3 target_position;                                                                // Position where this agent is currently heading towards.

        private CarController m_Car;                                                            // The car controller that we want to use.
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        public GameObject[] friends;
        public GameObject[] enemies;
        public Vector3 target_velocity;
        public Vector3 old_target_pos;
        public Vector3 desired_velocity;
        public float k_p = 2f;
        public float k_d = 0.5f;
        Rigidbody my_rigidbody;

        // ----- /Initializations -----

        private void Start()
        {

            m_Car = GetComponent<CarController>();                                              // Get the car controller
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            my_rigidbody = GetComponent<Rigidbody>();
            //old_target_pos = my_target.transform.position;

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
                //weighted_positions = new float[(int)(x_high - x_low), (int)(z_high - z_low)];
                float map_width = x_high - x_low;
                float map_height = z_high - z_low;
                int grid_width = (int)(map_width / grid_resolution);
                int grid_height = (int)(map_height / grid_resolution);

                weighted_positions = new float[grid_width, grid_height];
                //for (int x = (int)x_low; x < x_high; x++)
                //{
                //    for (int z = (int)z_low; z < z_high; z++)
                //    {
                for (int grid_i = 0; grid_i < grid_width; grid_i++)
                {
                    for (int grid_j = 0; grid_j < grid_height; grid_j++)
                    {
                        // Get x-coordinate of the middle of this grid position.
                        float x_step = (x_high - x_low) / grid_width;
                        float x = x_low + x_step / 2 + x_step * grid_i;

                        // Get z-coordinate of the middle of this grid position.
                        float z_step = (z_high - z_low) / grid_height;
                        float z = z_low + z_step / 2 + z_step * grid_j;

                        float score = 0.0f;                                                     // Base score is zero.

                        if (CheckObstacle(x, z) is false)                                       // Only check obstacle-free positions.
                        {
                            // ----- "Target in sight"-score -----
                            Vector3 position = new Vector3(x, 0.0f, z);

                            if (CheckInSight(position, current_target_position))                // If the current target is in sight.
                            {
                                //score += target_in_sight_score;                                 // Add a score if the target is in sight.         ##### Weird bug when using this #####
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
                        //int x_index = x - (int)x_low;                                           // Change from actual coordinates to indices in the matrix.
                        //int z_index = z - (int)z_low;

                        //Debug.Log("Score at x:" + x + ", z:" + z + " = " + score);

                        weighted_positions[grid_i, grid_j] = score;                           // Save the score at this position in the matrix.
                    }
                }
            }
            else
            {
                weighted_positions = friends[0].GetComponent<CarAI5>().weighted_positions;
            }
            // ----- /Build weighted position matrix -----


            // ----- Rotating Rigid Formation Path Finding -----
            List<Vector3> starting_formation = new List<Vector3>();                                 // TODO: Experiment with custom starting formation?
            if (this_id == 0)
            {
                foreach (GameObject friend in friends)
                {
                    starting_formation.Add(friend.transform.position);
                }
                starting_formation = SortFormation(starting_formation);
                traversal_graph = RRFPathFinding(starting_formation);
                Debug.Log("TEEEEEEEEEEEST");
                Debug.Log(traversal_graph);

                target_rf_reached = true;
                int target_rf_number = -1;
            }
            

            // ########################################################################################################################################################################
            // ######################## Hur hittar vi current_rf_number från startpositionen ?? ##############################################
            // ######################## Borde kunna använda startpositionen starting_formation och få fram det genom den (rotation 0)? ##############################################
            // ########################################################################################################################################################################

            // ----- /Rotating Rigid Formation Path Finding -----

            

            if (DEBUG && this_id == 0)
            {
                // ----- Print current target position in i,j grid -----
                //int target_pos_i = terrain_manager.myInfo.get_i_index(current_target_position[0]);
                //int target_pos_j = terrain_manager.myInfo.get_j_index(current_target_position[2]);
                //Debug.Log("Target grid position: i=" + target_pos_i + ", j=" + target_pos_j);
                // ----- /Print current target position in i,j grid -----


                // ----- Visualize the Rotated Rigid Formations -----
                //float position_1_x = terrain_manager.myInfo.get_x_pos(5);                       // Coordinates (5,9).
                //float position_1_z = terrain_manager.myInfo.get_z_pos(9);
                //Vector3 position_1 = new Vector3(position_1_x, 0.0f, position_1_z);
                //float position_2_x = terrain_manager.myInfo.get_x_pos(6);                       // Coordinates (6,9).
                //float position_2_z = terrain_manager.myInfo.get_z_pos(9);
                //Vector3 position_2 = new Vector3(position_2_x, 0.0f, position_2_z);
                //float position_3_x = terrain_manager.myInfo.get_x_pos(7);                       // Coordinates (7,9).
                //float position_3_z = terrain_manager.myInfo.get_z_pos(9);
                //Vector3 position_3 = new Vector3(position_3_x, 0.0f, position_3_z);

                ////Debug.Log(position_1);
                ////Debug.Log(position_2);
                ////Debug.Log(position_3);

                //List<Vector3> formation_pattern = new List<Vector3>();
                //formation_pattern.Add(position_1);
                //formation_pattern.Add(position_2);
                //formation_pattern.Add(position_3);

                //List<RigidFormation> possible_rigid_formations = GetRotatedRigidFormations(formation_pattern, rrf_resolution);

                ////Debug.Log("---------------------------------------------------");

                //foreach (RigidFormation rf in possible_rigid_formations)
                //{
                //    Vector3 leader = rf.agent_positions[0];
                //    foreach (Vector3 agent_pos in rf.agent_positions)
                //    {
                //        //Color color = Color.magenta;
                //        //Debug.DrawLine(leader, agent_pos, color, float.PositiveInfinity);
                //    }
                //}
                // ----- /Visualize the Rotated Rigid Formations -----
            }
        }

        private void FixedUpdate()
        {
            GameObject current_car = m_Car.gameObject;
            this_position = current_car.transform.position;                                     // Update current agent's position to current.

            // Execute your path here
            // ...

            //if (this_id == 0)
            //{
            //    Debug.Log(traversal_graph);
            //}

            // Find adjacent node with minimum edge weight.                                 TODO: Ändra till maximum?
            if (this_id == 0)
            {
                float min_weight = float.MaxValue;
                if (target_rf_reached)
                {
                    for (int i = 0; i < traversal_graph.num_vertices; i++)
                    {
                        if (traversal_graph.adj_matrix[current_rf_number, i] < min_weight)
                        {
                            target_rf_number = i;
                            min_weight = traversal_graph.adj_matrix[current_rf_number, i];
                        }
                    }
                    target_rf_reached = false;
                }
            }

            // ############################ This will not be correct atm. TODO: Sort the agent_positions in same order as this_id ############################

            

            target_position = this_position;
            //if (!target_rf_reached)
            //{
            //    List<Vector3> target_agent_positions = traversal_graph.rf_number_dictionary[target_rf_number].agent_positions;
            //    target_position = target_agent_positions[this_id];
            //}
            
            // ############## TODO: Set target_rf_reached to true when all agents are in position. ##############


            // ----- Petter's PD controller -----
            // keep track of target position and velocity
            //Vector3 target_position = null;
            target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            old_target_pos = target_position;

            // a PD-controller to get desired velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            // ----- /Petter's PD controller -----

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

        public RFGraph RRFPathFinding(List<Vector3> initial_formation_pattern)
        {
            // Rotating Rigid Formation Path Finding.
            multi_rf_dict = new Dictionary<int, RigidFormation>();
            List<List<Vector3>> possible_formation_patterns = GetRotatedFormationPatterns(initial_formation_pattern, rrf_resolution);
            float x;
            float z;
            float current_rotation = 0.0f;                                                                                  // Indicates current rotation angle.
            RFGraph rf_graph;
            RigidFormation rf;
            List<RFGraph> rf_graph_list = new List<RFGraph>();                                                              // List of all graphs, one graph per rotation.
            int num_vertices = weighted_positions.Length;                                                                   // Number of cells in the weighted grid.
            float formation_height;                                                                                         // Height of the current formation.
            float formation_width;                                                                                          // Width of the current formation.
            foreach (List<Vector3> formation_pattern in possible_formation_patterns)                                        // Iterate over each possible formation rotation, i.e each graph.
            {
                rf_graph = new RFGraph(current_rotation);                                                                   // Initialize a graph for the rigid formations at this rotation.
                x = x_low;                                                                                                  // Set the starting position to top left.
                z = z_high;                                                                                                 // ...
                List<Vector3> agent_positions = new List<Vector3>(formation_pattern);                                       // Make a copy of the formation pattern as the agent's position.
                //RigidFormation rf = new RigidFormation(rf_graph.rf_list.Count, formation_pattern, weighted_positions);    // Create a RigidFormation object of the formation pattern.
                formation_height = GetFormationHeight(agent_positions);
                formation_width = GetFormationWidth(agent_positions);
                for (int i = 0; i < z_high + formation_height; i++)                                                         // Iterate over map height + formation height.
                {
                    for (int j = 0; j < x_high + formation_width; j++)                                                      // Iterate over map width + formation width.
                    {
                        agent_positions = MoveFormation(agent_positions, new Vector3(x, 0.0f, z));                          // Move formation.
                        if (agent_positions[0][0] != -999)                                                                  // Check that the move was possible (i.e no agent hit an obstacle).
                        {
                            rf = new RigidFormation(rf_graph.rf_list.Count, agent_positions, weighted_positions);           // Create a RigidFormation object for the new position.
                            rf_graph.AddRigidFormation(rf.rf_number, x, z, rf);                                             // Add this rigid formation to the graph (note: without edges).
                        }
                        x += 1.0f;                                                                                          // Add one step to the right.
                    }
                    x = x_low;                                                                                              // Reset x to leftmost position.
                    z -= 1.0f;                                                                                              // Shift down one row.
                }
                // Link nodes in rf_graph with its adjacent vertices and set the weight of every outgoing edge to be the adjacent vertex's survivability score.
                rf_graph.InitAdjacencyMatrix(rf_graph.rf_list.Count);                                                       // Initialize the adjacency matrix so we can add edges.
                foreach (RigidFormation rigid_formation in rf_graph.rf_list)
                {
                    rigid_formation.ConnectNeighbors(rf_graph);                                                             // Connect this rf to adjacent rf's in this graph with survivability as edge weight.
                }

                current_rotation += rrf_resolution;                                                                         // Update next rotation angle.
                rf_graph_list.Add(rf_graph);                                                                                // Add the current graph to list of graphs that will later be connected.
            }

            // Link all rf_graphs to a single multigraph.
            RFGraph multi_graph = ConnectGraphs(rf_graph_list);
            return multi_graph;
        }

        public List<Vector3> SortFormation(List<Vector3>  agent_positions)
        {
            // ########################################################################################################################################################################
            // ######################## Ändra så att den sorterar efter this_id?? Så att man kan få fram vilken pos varje agent ska till ##############################################
            // ########################################################################################################################################################################
            agent_positions.Sort((x, y) => {                                                                                // Sort by x-position first and then by y-position.
                var ret = x[0].CompareTo(y[0]);
                if (ret == 0) ret = x[2].CompareTo(y[2]);
                return ret;
            });
            return agent_positions;
        }

        public float GetFormationWidth(List<Vector3> agent_positions)
        {
            // Returns the horisontal size of the formation.
            float x_min = float.PositiveInfinity;
            float x_max = float.NegativeInfinity;
            foreach (Vector3 agent_pos in agent_positions)
            {
                x_min = Math.Min(x_min, agent_pos[0]);
                x_max = Math.Max(x_max, agent_pos[0]);
            }
            return x_max - x_min;
        }

        public float GetFormationHeight(List<Vector3> agent_positions)
        {
            // Returns the vertical size of the formation.
            float z_min = float.PositiveInfinity;
            float z_max = float.NegativeInfinity;
            foreach (Vector3 agent_pos in agent_positions)
            {
                z_min = Math.Min(z_min, agent_pos[2]);
                z_max = Math.Max(z_max, agent_pos[2]);
            }
            return z_max - z_min;
        }

        public List<List<Vector3>> GetRotatedFormationPatterns(List<Vector3> formation_pattern, int rotation_resolution)
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

        public List<Vector3> MoveFormation(List<Vector3> agent_positions, Vector3 new_leader_position)
        {
            // Moves the formation. Returns new positions of agents if successful and null if an agent ends up in an obstacle.
            Vector3 old_leader_position = agent_positions[0];
            Vector3 position_difference = new_leader_position - old_leader_position;
            List<Vector3> new_agent_positions = new List<Vector3>(agent_positions);
            for (int i = 0; i < agent_positions.Count; i++)
            {
                new_agent_positions[i] += position_difference;
                if (CheckObstacle(new_agent_positions[i][0], new_agent_positions[i][2]))
                {
                    // Set the positions to -999 (temporary solution instead of nullable types).
                    for (int j = 0; j < agent_positions.Count; j++)
                    {
                        new_agent_positions[j] = new Vector3(-999, -999, -999);
                    }
                    return new_agent_positions;
                }
            }
            return new_agent_positions;
        }

        public RFGraph ConnectGraphs(List<RFGraph> rf_graph_list)
        {
            // Mutually link vertices from rf_graph[i] containing joint agents that are also placed on the same positions in vertices from rf_graph[i+1].
            RFGraph multi_graph = new RFGraph(0);
            int num_vertices = 0;                                                                                           // Number of vertices in the multigraph.
            List<int> index_offset = new List<int>();                                                                       // List of offset for each subgraph for the adjacency matrix in the multigraph.
            foreach (RFGraph rf_graph in rf_graph_list)
            {
                index_offset.Add(num_vertices);                                                                             // Number of vertices before this rf_graph to be used as offset.
                num_vertices += rf_graph.rf_list.Count;                                                                     // Number of rigid formations (vertices) in this subgraph.
            }

            multi_graph.InitAdjacencyMatrix(num_vertices);                                                                  // Create an adjacency matrix with vertices from every subgraph.

            

            for (int i = 0; i < rf_graph_list.Count; i++)                                                                   // Iterate over each rf_graph.
            {
                foreach (RigidFormation rf1 in rf_graph_list[i].rf_list)
                {
                    int rf1_number_multi = rf1.rf_number + index_offset[i];                                                 // Offset the rf's number from rf_graph[i].
                    multi_rf_dict.Add(rf1_number_multi, rf1);                                                               // Add this rigid formation object in a dictionary.
                    foreach (RigidFormation rf2 in rf_graph_list[i+1].rf_list)
                    {
                        if (rf1.Equals(rf2))                                                                                // Check if the agent's positions matches.
                        {
                            
                            int rf2_number_multi = rf2.rf_number + index_offset[i + 1];                                     // Offset the rf's number from rf_graph[i+1].
                            multi_graph.AddEdge(rf1_number_multi, rf2_number_multi, 0);                                     // Add edge from the node in rf_graph[i] to the same node in rf_graph[i+1] with no weight.
                        }
                    }
                }
            }
            return multi_graph;
        }

        public float[] Dijkstra(RFGraph rf_graph, int source)
        {
            float[,] graph = rf_graph.adj_matrix;
            float[] distances = new float[rf_graph.num_vertices];
            bool[] visited = new bool[rf_graph.num_vertices];
            for (int i = 0; i < rf_graph.num_vertices; i++)
            {
                distances[i] = float.MaxValue;
                visited[i] = false;
            }
            distances[source] = 0;

            for (int count = 0; count < rf_graph.num_vertices - 1; count++)
            {
                float min = float.MaxValue;
                int u = -1;

                // Find unvisited node u with shortest distance.
                for (int v = 0; v < rf_graph.num_vertices; v++)
                {
                    if (visited[v] == false && distances[v] <= min)
                    {
                        min = distances[v];
                        u = v;
                    }
                }
                visited[u] = true;

                for (int v = 0; v < rf_graph.num_vertices; v++)
                {
                    if (!visited[v] && graph[u, v] != 0 && distances[u] != float.MaxValue && distances[u] + graph[u, v] < distances[v])
                    {
                        distances[v] = distances[u] + graph[u, v];
                    }
                }
            }
            return distances;
        }

        //public void OnDrawGizmos()
        //{
        //    // Debugging in Scene view.
        //    if (DEBUG && this_id == 0)
        //    {
        //        Handles.Label(this_position, "ID: " + this_id);                                                                     // Show ID of the cars.

        //        Gizmos.DrawSphere(new Vector3(x_low, 10.0f, z_high), 5);                                                            // Visualize a point.

        //        // ----- Create and visualize Rotated Rigid Formations -----
        //        //float position_1_x = terrain_manager.myInfo.get_x_pos(5);                       // Coordinates (5,9).
        //        //float position_1_z = terrain_manager.myInfo.get_z_pos(9);
        //        //Vector3 position_1 = new Vector3(position_1_x, 0.0f, position_1_z);
        //        //float position_2_x = terrain_manager.myInfo.get_x_pos(6);                       // Coordinates (6,9).
        //        //float position_2_z = terrain_manager.myInfo.get_z_pos(9);
        //        //Vector3 position_2 = new Vector3(position_2_x, 0.0f, position_2_z);
        //        //float position_3_x = terrain_manager.myInfo.get_x_pos(7);                       // Coordinates (7,9).
        //        //float position_3_z = terrain_manager.myInfo.get_z_pos(9);
        //        //Vector3 position_3 = new Vector3(position_3_x, 0.0f, position_3_z);

        //        //List<Vector3> formation_pattern = new List<Vector3>();
        //        //formation_pattern.Add(position_1);
        //        //formation_pattern.Add(position_2);
        //        //formation_pattern.Add(position_3);

        //        //Gizmos.color = Color.blue;
        //        //GUIStyle style_original = new GUIStyle();
        //        //style_original.normal.textColor = Color.blue;
        //        //Gizmos.DrawSphere(position_1, 1);
        //        //Handles.Label(position_1 + Vector3.back * 3 + Vector3.left * 6, "" + position_1, style_original);
        //        //Gizmos.DrawSphere(position_2, 1);
        //        //Handles.Label(position_2 + Vector3.back * 3 + Vector3.left * 6, "" + position_2, style_original);
        //        //Gizmos.DrawSphere(position_3, 1);
        //        //Handles.Label(position_3 + Vector3.back * 3 + Vector3.left * 6, "" + position_3, style_original);

        //        //List<RigidFormation> possible_rigid_formations = GetRotatedRigidFormations(formation_pattern, rrf_resolution);

        //        //foreach (RigidFormation rf in possible_rigid_formations)
        //        //{
        //        //    Vector3 leader = rf.agent_positions[0];
        //        //    int f_index = 0;
        //        //    foreach (Vector3 agent_pos in rf.agent_positions)
        //        //    {
        //        //        Gizmos.color = Color.magenta;
        //        //        Gizmos.DrawSphere(agent_pos, 1);
        //        //        Handles.Label(agent_pos + Vector3.back + Vector3.left * 6, "" + f_index + ". " + agent_pos);
        //        //        f_index++;
        //        //    }
        //        //}
        //        // ----- /Visualize the Rotated Rigid Formations -----


        //        // ----- Draw scores in a grid fashion -----
        //        int x_N = terrain_manager.myInfo.x_N;
        //        int z_N = terrain_manager.myInfo.z_N;
        //        Vector3 position;
        //        float score;
        //        float grid_center_x;
        //        float grid_center_z;
        //        GUIStyle style = new GUIStyle();
        //        style.normal.textColor = Color.green;
        //        for (int i = 0; i < x_N; i++)
        //        {
        //            for (int j = 0; j < z_N; j++)
        //            {
        //                grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        //                grid_center_z = terrain_manager.myInfo.get_z_pos(j);
        //                position = new Vector3(grid_center_x, 0.0f, grid_center_z);
        //                score = weighted_positions[(int)Math.Floor(grid_center_x - x_low), (int)Math.Floor(grid_center_z - z_low)];
        //                Handles.Label(position, "" + score, style);
        //                //Handles.Label(position, "" + i + "," + j, style);
        //            }
        //        }
        //        // ----- /Draw scores in a grid fashion -----
        //    }
        //}
}


    public class RigidFormation
    {
        public int rf_number;                                                                                   // Unique number for this rigid formation in its graph (from 0 to number of rf's).
        public float[,] weighted_positions;
        public int nr_of_agents;
        public List<Vector3> agent_positions;
        

        public RigidFormation(int rf_number, List<Vector3> agent_positions, float[,] weighted_positions)
        {
            this.rf_number = rf_number;
            this.nr_of_agents = agent_positions.Count;
            this.agent_positions = agent_positions;
            agent_positions.Sort((x, y) => {                                                                    // Sort by x-position first and then by y-position.     TODO: Check if it works.
                var ret = x[0].CompareTo(y[0]);                                                                 // TODO: Is sorting necessary?
                if (ret == 0) ret = x[2].CompareTo(y[2]);
                return ret;
            });
            this.weighted_positions = weighted_positions;
        }

        public Vector3 GetLeaderPosition()
        {
            return this.agent_positions[0];
        }

        public float GetSurvivability()
        {
            // Returns the survivability of the formation at their current positions.
            // Input: Grid with survivability score at each position.
            float formation_survivability = 0;
            foreach (Vector3 agent_pos in this.agent_positions)
            {
                formation_survivability += this.weighted_positions[(int)agent_pos[0], (int)agent_pos[2]];        // TODO: Change from addition to something else (?).
            }
            return formation_survivability;
        }

        public void ConnectNeighbors(RFGraph rf_graph)
        {
            // Adds directed edges from this rigid formation to its adjacent neighbors, with the neighbor's survivability score as edge weight.
            // Above:   (x - 1, z - 1), (x, z - 1), (x + 1, z - 1);
            // Sides:   (x - 1, z), (x + 1, z);
            // Below:   (x - 1, z + 1), (x, z + 1), (x + 1, z + 1);
            float leader_x = this.agent_positions[0][0];                                                                            // x-coordinate of this rigid formation's leader.
            float leader_z = this.agent_positions[0][2];                                                                            // z-coordinate of this rigid formation's leader.
            (float, float)[] positions_around = new (float, float)[] { (leader_x - 1, leader_z - 1), (leader_x, leader_z - 1), (leader_x + 1, leader_z - 1),    // Above.
                (leader_x - 1, leader_z), (leader_x + 1, leader_z),                                                                                             // Sides.
                (leader_x - 1, leader_z + 1), (leader_x, leader_z + 1), (leader_x + 1, leader_z + 1)};                                                          // Below.
            foreach ((float, float) position in positions_around)                                                                   // Iterate over adjacent positions including diagonal positions.
            {
                if (rf_graph.rf_pos_dictionary.TryGetValue((position.Item1, position.Item2), out RigidFormation rf_neighbor))       // Checks if there is a neighbor at this position.
                {
                    rf_graph.AddEdge(this.rf_number, rf_neighbor.rf_number, rf_neighbor.GetSurvivability());                        // Add edge with neighbor's survivability score as weight.
                }
            }
        }

        public override bool Equals(object obj)
        {
            // Compare two rigid formations if they have all agents at the exact same position (including order).               TODO: Should order be taken into account or not?
            // Note: Two rigid formations should only be equal if they belong to separate graphs (i.e different rotation of the formation pattern).
            if ((obj == null) || !this.GetType().Equals(obj.GetType()))                                     // Check for null and compare run-time types.
            {
                return false;
            }
            else
            {
                RigidFormation other_rf = (RigidFormation)obj;
                for (int i = 0; i < this.nr_of_agents; i++)
                {
                    if (this.agent_positions[i] != other_rf.agent_positions[i])
                    {
                        return false;
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


    //public class RFNode
    //{
    //    //public static Dictionary<(float, float, float), RFNode> node_dictionary = new Dictionary<(float, float, float), RFNode>();     // Dictionary of all nodes with (x, z, rotation) tuple as key.
    //    private static float cell_size = 1.0f;                                              // Size of the cells in the grid. Should correspond to the grid resolution.
    //    public float x;
    //    public float z;
    //    public float rotation;
    //    public float survivability;                                                         // Measure of team survivability at this position.
    //    public int rf_node_nr;                                                        
    //    private List<RFNode> neighbors = new List<RFNode>();                                // List of adjacent non-obstacle nodes.

    //    public RFNode(RFGraph rf_graph, float x, float z, float rotation, float survivability, int rf_node_nr)
    //    {
    //        this.x = x;
    //        this.z = z;
    //        this.rotation = rotation;
    //        this.survivability = survivability;
    //        this.rf_node_nr = rf_node_nr;
    //        rf_graph.AddNode((this.x, this.z), this);                                       // Add this node to a dictionary in the graph.
    //    }

    //    public bool CheckAdjacent(RFNode node)
    //    {
    //        // Checks if this node is adjacent to or on the same position as the input node.
    //        if (node.x == this.x || Math.Abs(node.x - this.x) == cell_size)                 // Same or adjacent x position.
    //        {
    //            if (node.z == this.z || Math.Abs(node.z - this.z) == cell_size)             // Same or adjacent z position.
    //            {
    //                return true;
    //            }
    //        }
    //        return false;
    //    }

    //    public void AddNeighbor(RFNode adjacent_node)
    //    {
    //        this.neighbors.Add(adjacent_node);
    //    }

    //    //public static RFNode GetNode(float x, float z, float rotation)                // Moved to RFGraph.
    //    //{
    //    //    // Returns the node object given x and z coordinates and rotation.
    //    //    return node_dictionary[(x, z, rotation)];
    //    //}
    //}


    public class RFGraph
    {
        //public Dictionary<(float, float, float), RFNode> node_dictionary = new Dictionary<(float, float, float), RFNode>();     // Dictionary of all nodes with (x, z, rotation) tuple as key.
        public Dictionary<(float, float), RigidFormation> rf_pos_dictionary;            // Dictionary of all rigid formations with the leader's position as key.
        public Dictionary<int, RigidFormation> rf_number_dictionary;                    // Dictionary of all rigid formations with the rf's unique number as key.
        public List<RigidFormation> rf_list;                                            // List of all rigid formations.
        public float[,] adj_matrix;
        public float rotation;
        public int num_vertices;

        public RFGraph(float rotation)
        {
            this.rotation = rotation;
            this.rf_pos_dictionary = new Dictionary<(float, float), RigidFormation>();
            this.rf_number_dictionary = new Dictionary<int, RigidFormation>();
            this.rf_list = new List<RigidFormation>();
        }

        //public void AddNode(float x, float z, RFNode node)
        //{
        //    this.node_dictionary.Add((x, z, rotation), node);
        //}

        public void AddRigidFormation(int rf_number, float x, float z, RigidFormation rf)
        {
            // Store the RigidFormation objects in the graph.
            this.rf_pos_dictionary.Add((x, z), rf);
            this.rf_number_dictionary.Add(rf_number, rf);
            this.rf_list.Add(rf);
        }

        public void InitAdjacencyMatrix(int num_vertices)
        {
            // Initializes the adjacency matrix for this graph.
            this.num_vertices = num_vertices;
            Debug.Log("RFGraph number of vertices: " + num_vertices);
            this.adj_matrix = new float[num_vertices, num_vertices];
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

        //public void GetAdjacent(int node)
        //{
        //    for (int i = 0; i < num_vertices; i++)
        //    {

        //    }
        //}
    }
}
