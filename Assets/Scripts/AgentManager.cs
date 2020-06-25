using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class AgentManager : MonoBehaviour
{
    public int agentCount = 10;
    public float agentSpawnRadius = 20;
    public GameObject agentPrefab;
    public static Dictionary<GameObject, Agent> agentsObjs = new Dictionary<GameObject, Agent>();

    private static List<Agent> agents = new List<Agent>();
    private GameObject agentParent;
    public Vector3 destination;
    public static AgentManager instance;
    
    private const int PATHFINDING_FRAME_SKIP = 25;

    #region Unity Functions

    void Awake()
    {
        instance = this;

        Random.InitState(0);

        agentParent = GameObject.Find("Agents");
        for (int i = 0; i < agentCount; i++)
        {
            var randPos = new Vector3((Random.value - 0.5f) * agentSpawnRadius, 0, (Random.value - 0.5f) * agentSpawnRadius);
            NavMeshHit hit;
            NavMesh.SamplePosition(randPos, out hit, 10, NavMesh.AllAreas);
            randPos = hit.position + Vector3.up;

            GameObject agent = null;
            agent = Instantiate(agentPrefab, randPos, Quaternion.identity);
            agent.name = "Agent " + i;
            agent.transform.parent = agentParent.transform;
            var agentScript = agent.GetComponent<Agent>();
            agentScript.radius = 0.3f;// Random.Range(0.2f, 0.6f);
            agentScript.mass = 80;
            agentScript.perceptionRadius = 3;

            agents.Add(agentScript);
            agentsObjs.Add(agent, agentScript);
        }

        StartCoroutine(Run());
    }
    
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            var point = Camera.main.ScreenToWorldPoint(Input.mousePosition + Vector3.forward * 10);
            var dir = point - Camera.main.transform.position;
            RaycastHit rcHit;
            if (Physics.Raycast(point, dir, out rcHit))
            {
                point = rcHit.point;

                destination = point;
            }
        }
    }

    IEnumerator Run()
    {
        yield return null;

        for (int frames = 0; ; frames++)
        {
            if (frames % PATHFINDING_FRAME_SKIP == 0)
            {
                SetAgentDestinations(destination);
            }

            foreach (var agent in agents)
            {
                agent.ApplyForce();
            }

            yield return null;
        }
    }

    #endregion

    #region Public Functions

    public static bool IsAgent(GameObject obj)
    {
        return agentsObjs.ContainsKey(obj);
    }

    public void SetAgentDestinations(Vector3 destination)
    {
        NavMeshHit hit;
        NavMesh.SamplePosition(destination, out hit, 10, NavMesh.AllAreas);
        foreach (var agent in agents)
        {
            agent.ComputePath(hit.position);
        }
    }

    public static void RemoveAgent(GameObject obj)
    {
        var agent = obj.GetComponent<Agent>();

        agents.Remove(agent);
        agentsObjs.Remove(obj);
    }

    #endregion

    #region Private Functions

    #endregion

    #region Visualization Functions

    #endregion

}
