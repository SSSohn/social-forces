using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();
    private HashSet<GameObject> adjacentWalls = new HashSet<GameObject>();

    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (false)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (false)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }
        
        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force / mass, ForceMode.Acceleration);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            perceivedNeighbors.Add(other.gameObject);
        }
        if (WallManager.IsWall(other.gameObject))
        {
            adjacentWalls.Add(other.gameObject);
        }
    }

    public void OnTriggerExit(Collider other)
    {
        if (perceivedNeighbors.Contains(other.gameObject))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
        if (adjacentWalls.Contains(other.gameObject))
        {
            adjacentWalls.Remove(other.gameObject);
        }
    }

    #endregion

    #region Private Functions

    private Vector3 ComputeForce()
    {
        var force = CalculateGoalForce() + CalculateAgentForce() + CalculateWallForce();

        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.MAX_ACCEL);
        } else
        {
            return Vector3.zero;
        }
    }
    
    private Vector3 CalculateGoalForce()
    {
        if (path.Count == 0)
        {
            return Vector3.zero;
        }

        var temp = path[0] - transform.position;
        var desiredVel = temp.normalized * Parameters.DESIRED_SPEED;
        var actualVelocity = rb.velocity;
        return mass * (desiredVel - actualVelocity) / Parameters.T;
    }

    private Vector3 CalculateAgentForce()
    {
        var agentForce = Vector3.zero;

        foreach (var n in perceivedNeighbors)
        {
            if (!AgentManager.IsAgent(n))
            {
                continue;
            }

            var neighbor = AgentManager.agentsObjs[n];
            var dir = (transform.position - neighbor.transform.position).normalized;
            var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);

            agentForce += Parameters.A * Mathf.Exp(overlap / Parameters.B) * dir;
            agentForce += Parameters.K * (overlap > 0f ? 1 : 0) * dir;

            var tangent = Vector3.Cross(Vector3.up, dir);
            agentForce += Parameters.KAPPA * (overlap > 0f ? overlap : 0) * Vector3.Dot(rb.velocity - neighbor.GetVelocity(), tangent) * tangent;
        }

        return agentForce;
    }

    private Vector3 CalculateWallForce()
    {
        var wallForce = Vector3.zero;

        foreach (var wall in adjacentWalls)
        {
            var wallCentroid = wall.transform.position;
            var pos = transform.position;

            #region Compute Normal

            var normal = pos - wallCentroid;
            normal.y = 0;

            if (Mathf.Abs(normal.x) > Mathf.Abs(normal.z))
            {
                normal.z = 0;
            }
            else
            {
                normal.x = 0;
            }
            normal.Normalize();

            #endregion

            var dir = (pos - wallCentroid);
            dir.y = 0;
            var agentToWallProj = Vector3.Project(dir, normal);
            var overlap = (radius + 0.5f) - agentToWallProj.magnitude;

            wallForce += Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B) * normal;
            wallForce += Parameters.WALL_K * (overlap > 0f ? 1 : 0) * dir;

            var tangent = Vector3.Cross(Vector3.up, normal);
            wallForce += Parameters.WALL_KAPPA * (overlap > 0f ? overlap : 0) * Vector3.Dot(rb.velocity, tangent) * tangent;
        }

        return wallForce;
    }
    
    #endregion
}
