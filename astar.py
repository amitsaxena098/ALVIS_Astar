from queue import PriorityQueue

class Node():
	def __init__(self, node, edwt):
		self.node = node
		self.edwt = edwt
class astar(AlgorithmBase):
	def execute(self):
		start=self.start_nodes[0]
		goal=self.goal_nodes[0]
		queue, visited = self.get_list('open'), self.get_list('closed')
		w = 1
		cost = float('inf')
		best_path = ""
		while True:
			queue.append(start)
			node = Node(start, 0)
			pq = PriorityQueue()
			pq.put((0, node))
			curr_cost = 0
			while queue:
				self.alg_iteration_start()
				noder = pq.get()
				if(noder[1].node != start):
					curr_cost =  noder[1].edwt
				queue.remove(noder[1].node)
				visited.append(noder[1].node)
				if noder[1].node == goal:
					visited.append(noder[1].node)
					self.found_goal = True
					break
				for neighbor in self.neighbors(noder[1].node):
					f = (noder[1].edwt + self.get_edge_weight(noder[1].node, neighbor)) + w * self.heuristic(neighbor, goal)
					if neighbor not in visited and neighbor not in queue:
						self.set_parent(neighbor, noder[1].node)
						queue.append(neighbor)
						tmp = Node(neighbor, noder[1].edwt+self.get_edge_weight(noder[1].node, neighbor))
						pq.put((f, tmp))
					elif neighbor in queue:
						pq_tmp = []
						node_tmp = pq.get()
						while node_tmp[1].node != neighbor and not pq.empty():
							pq_tmp.append(node_tmp)
							node_tmp = pq.get()
						if node_tmp[1].node == neighbor and f < node_tmp[0]:
							self.set_parent(neighbor, noder[1].node)
							tmp = Node(neighbor, noder[1].edwt+self.get_edge_weight(noder[1].node, neighbor))
							pq.put((f, tmp))
						else:
							pq.put(node_tmp)
						while pq_tmp:
							pq.put(pq_tmp.pop(0))
				self.alg_iteration_end()
			if self.found_goal:
				path = self.gen_path()
				if( cost > curr_cost):
					cost = curr_cost
					best_path = path
				self.show_path(best_path)
			else:
				self.show_info("No path found")
				curr_cost = 0
			self.show_info("Next iteration with w = " + str(w) + "\nBest cost = " + str(cost) + "\nCurrent cost = " + str(curr_cost))
			queue.clear()
			visited.clear()
			w = w+1
