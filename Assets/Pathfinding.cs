using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


public class Pathfinding : MonoBehaviour {

	public Transform seeker, target;
	Grid grid;
	int count1 = 0;
	int count2 = 0;
	int count3 = 0;
	int count4 = 0;

	void Awake()
	{
		grid = GetComponent<Grid>();
	}

	void Update()
	{
		count1 = 0;
		count2 = 0;
		count3 = 0;
		count4 = 0;

		var watch1 = new System.Diagnostics.Stopwatch();
		var watch2 = new System.Diagnostics.Stopwatch();
		var watch3 = new System.Diagnostics.Stopwatch();
		var watch4 = new System.Diagnostics.Stopwatch();

		watch1.Start();
		FindPath(seeker.position, target.position);
		watch1.Stop();
		Debug.Log($"Execution Time Euclidian: {watch1.ElapsedMilliseconds * 100} ms, retracement : {count1}");

		watch2.Start();
		FindPathUCS(seeker.position, target.position);
		watch2.Stop();
		Debug.Log($"Execution Time Euclidian: {watch2.ElapsedMilliseconds * 100} ms, retracement : {count2}");

		watch3.Start();
		FindPathBFS(seeker.position, target.position);
		watch3.Stop();
		Debug.Log($"Execution Time Euclidian: {watch3.ElapsedMilliseconds * 100} ms, retracement : {count3}");

		watch4.Start();
		FindPathDFS(seeker.position, target.position);
		watch4.Stop();
		Debug.Log($"Execution Time Euclidian: {watch4.ElapsedMilliseconds * 100} ms, retracement : {count4}");

		FindPath (seeker.position, target.position);
		FindPathUCS (seeker.position, target.position);
		FindPathBFS (seeker.position, target.position);
		FindPathDFS(seeker.position, target.position);
	}


	void FindPath(Vector3 startPos, Vector3 targetPos)
	{
		var watch = new System.Diagnostics.Stopwatch();
		watch.Start();

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePath(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		watch.Stop();
		Debug.Log($"Execution Time Euclidian: {watch.ElapsedMilliseconds} ms");
	}


	void FindPathUCS(Vector3 startPos, Vector3 targetPos)
	{
		var watch = new System.Diagnostics.Stopwatch();
		watch.Start();

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].gCost < node.gCost || openSet[i].gCost == node.gCost)
				{
					node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathUCS(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance2(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		} 
		watch.Stop();
		Debug.Log($"Execution Time Manhattan: {watch.ElapsedMilliseconds} ms");
	}


	void FindPathBFS(Vector3 startPos, Vector3 targetPos)
	{
		var watch = new System.Diagnostics.Stopwatch();
		watch.Start();

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		Queue<Node> queueBFS = new Queue<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		queueBFS.Enqueue(startNode);

		while (queueBFS.Count != 0)
		{
			Node currentNode = queueBFS.Dequeue();
			if (currentNode == targetNode)
			{
				RetracePathBFS(startNode, targetNode);
				return;
			}
			closedSet.Add(currentNode);
			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}
				if (neighbour.walkable || !queueBFS.Contains(neighbour))
				{
					closedSet.Add(neighbour);
					neighbour.parent = currentNode;
					queueBFS.Enqueue(neighbour);
				}
			}
		}
		watch.Stop();
		Debug.Log($"Execution Time Manhattan: {watch.ElapsedMilliseconds} ms");
	}



	void FindPathDFS(Vector3 startPos, Vector3 targetPos)
	{
		var watch = new System.Diagnostics.Stopwatch();
		watch.Start();

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		Stack<Node> StackDFS = new Stack<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		StackDFS.Push(startNode);

		while (StackDFS.Count != 0)
		{
			Node currentNode = StackDFS.Pop();
			if (currentNode == targetNode)
			{
				RetracePathDFS(startNode, targetNode);
				return;
			}
			closedSet.Add(currentNode);
			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}
				if (neighbour.walkable || !StackDFS.Contains(neighbour))
				{
					closedSet.Add(neighbour);
					neighbour.parent = currentNode;
					StackDFS.Push(neighbour);
				}
			}
		}
		watch.Stop();
		Debug.Log($"Execution Time Manhattan: {watch.ElapsedMilliseconds} ms");
	}


	void RetracePath(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			count1++;
		}
		path.Reverse();
		grid.path = path;
	}


	void RetracePathUCS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			count2++;
		}
		path.Reverse();
		grid.pathUCS = path;
	}


	void RetracePathBFS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			count3++;
		}
		path.Reverse();
		grid.pathBFS = path;
	}


	void RetracePathDFS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			count4++;
		}
		path.Reverse();
		grid.pathDFS = path;
	}


	int GetDistance(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		return dstX + dstY;
	}


	int GetDistance2(Node nodeA, Node nodeB)
	{
		return (int)Mathf.Sqrt((Mathf.Pow(nodeA.gridX - nodeB.gridX, 2) + Mathf.Pow(nodeA.gridY - nodeB.gridY, 2)));
	}
}