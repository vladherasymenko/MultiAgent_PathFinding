import mesa
import random
from queue import PriorityQueue

random.seed(1488)


class Obstacle(mesa.Agent):  # Un obstacle n'est pas vraiment un agent : c'est fait pour des raisons pratiques
    def __init__(self, unique_id, x, y, model):
        super().__init__(unique_id, model)
        self.x = x
        self.y = y
        self.type = "Obstacle"


class Target(mesa.Agent):  # Pareil pour la cible
    def __init__(self, unique_id, x, y, master_agent, model):
        super().__init__(unique_id, model)
        self.x = x
        self.y = y
        self.master_agent = master_agent
        self.type = "Target"


class PathAgent(mesa.Agent):

    def __init__(self, unique_id, x, y, destination, model):
        super().__init__(unique_id, model)
        self.x = x
        self.y = y
        self.destination = destination
        self.type = "Agent"
        self.display_color = "Red"
        self.path = []
        self.at_destination = False

    def step(self):
        if not self.at_destination:
            if self.path == [] or self.model.algo_type == 1:
                dist, path = self.model.dijkstra((self.x, self.y), self.destination)
                self.path = path

            next_step = self.path[0]
            for agent in self.model.agents:
                if agent.unique_id == self.unique_id:
                    continue
                if (agent.x, agent.y) == next_step:  # collision est possible avec un agent
                    if self.model.algo_type == 2:
                        self.path = []
                    return
                if agent.path != []:
                    if agent.path[0] == next_step:
                        if self.model.algo_type == 2:
                            # faire un pas aléatoire pour essayer d'éviter le bloquage
                            self.make_random_move()
                            self.path = []
                        if self.model.algo_type == 1:
                            self.path = []
                        return
            self.x = next_step[0]
            self.y = next_step[1]
            self.model.grid.move_agent(self, (self.x, self.y))

            if self.model.algo_type != 1:
                self.path = self.path[1:]

            if (self.x, self.y) == self.destination:
                self.at_destination = True

    def make_random_move(self):
        possible_positions = [(self.x + 1, self.y), (self.x - 1, self.y), (self.x, self.y + 1), (self.x, self.y - 1)]
        random.shuffle(possible_positions)
        for pos in possible_positions:
            is_acceptable = True
            for obst in self.model.obstacles:
                if obst.x == pos[0] and obst.y == pos[1]:
                    is_acceptable = False
                    break
            if is_acceptable:
                for agent in self.model.agents:
                    if agent.unique_id == self.unique_id:
                        continue
                    if (agent.x, agent.y) == pos:
                        is_acceptable = False
                        break
                    if agent.path != []:
                        if agent.path[0] == pos:
                            is_acceptable = False
                            break
            if is_acceptable:
                self.x = pos[0]
                self.y = pos[1]
                self.model.grid.move_agent(self, (self.x, self.y))
                self.path = self.path[1:]
                if (self.x, self.y) == self.destination:
                    self.at_destination = True


def count_avg_step_number(model):
    for agent in model.agents:
        if not agent.at_destination:
            model.avg_steps += (1/len(model.agents))


class PathModel(mesa.Model):

    def __init__(self, width, num_agents, num_obstacles, algo_type):
        super().__init__()

        self.algo_type = algo_type
        self.avg_steps = 0

        self.width = width
        self.height = width
        self.grid = mesa.space.MultiGrid(self.width, self.height, True)

        self.num_agents = num_agents
        self.num_obstacles = num_obstacles
        self.obstacles = []
        self.agents = []

        self.schedule = mesa.time.RandomActivation(self)  # TODO : Пріоритети

        self.fill_board()

        self.nodes = []
        self.construct_nodes()
        self.adjacency = [[-1 for i in range(len(self.nodes))] for j in range(len(self.nodes))]
        self.define_adjacency()
        self.add_agents()

        self.datacollector = mesa.DataCollector(
            model_reporters={"TOTAL": count_avg_step_number}
        )

    def fill_board(self):
        for j in range(self.num_obstacles):
            obst = Obstacle(j + 300, random.randint(1, self.width - 2), random.randint(1, self.height - 2), self)
            self.obstacles.append(obst)
            self.grid.place_agent(obst, (obst.x, obst.y))
        for i in range(self.width):
            obst1 = Obstacle(i + 100, 0, i, self)
            self.grid.place_agent(obst1, (obst1.x, obst1.y))
            obst2 = Obstacle(i + 200, self.width - 1, i, self)
            self.grid.place_agent(obst2, (obst2.x, obst2.y))
            obst3 = Obstacle(i + 300, i, self.width - 1, self)
            self.grid.place_agent(obst3, (obst3.x, obst3.y))
            obst4 = Obstacle(i + 400, i, 0, self)
            self.grid.place_agent(obst4, (obst4.x, obst4.y))
            self.obstacles.append(obst1)
            self.obstacles.append(obst2)
            self.obstacles.append(obst3)
            self.obstacles.append(obst4)

    def define_adjacency(self):
        for i in range(len(self.nodes)):
            for j in range(len(self.nodes)):
                node1 = self.nodes[i]
                node2 = self.nodes[j]
                if ((node2 == (node1[0], node1[1] + 1)  # si ce noeud2 est le voisin de noeud1
                     or node2 == (node1[0], node1[1] - 1)
                     or node2 == (node1[0] + 1, node1[1])
                     or node2 == (node1[0] - 1, node1[1]))):
                    can_place = True
                    if not self.grid.is_cell_empty(node2):  # check si la case est vide
                        can_place = False
                        for agent in self.agents:
                            if agent.destination == node2:
                                can_place = True
                                break
                    for agent in self.agents:  # check s'il y a des agents qui sont sur leur place
                        if agent.x == node2[0] and agent.y == node2[1] and agent.at_destination or \
                                agent.x == node1[0] and agent.y == node1[1] and agent.at_destination:
                            can_place = False
                    if can_place:
                        self.adjacency[i][j] = 1
                        self.adjacency[j][i] = 1

    def add_agents(self):
        for i in range(1, self.num_agents + 1):  # TODO : Перевірити, чи є вільні місця
            found_free_space = False
            while found_free_space is False:
                pos = (random.randint(1, self.width - 2), random.randint(1, self.height - 2))
                pos_target = (random.randint(1, self.width - 2), random.randint(1, self.height - 2))
                if self.grid.is_cell_empty(pos) and self.grid.is_cell_empty(pos_target):
                    a = PathAgent(i, pos[0], pos[1], pos_target, self)
                    self.schedule.add(a)
                    self.agents.append(a)

                    r = lambda: random.randint(0, 255)
                    agent_color = '#%02X%02X%02X' % (r(), r(), r())
                    a.display_color = agent_color

                    target = Target(500 + i, pos_target[0], pos_target[1], a, self)
                    self.grid.place_agent(a, (a.x, a.y))
                    self.grid.place_agent(target, (target.x, target.y))
                    found_free_space = True
                else:
                    continue

    def construct_nodes(self):
        for i in range(self.width):
            for j in range(self.height):
                pos = (i, j)
                if self.grid.is_cell_empty(pos):
                    self.nodes.append(pos)

    def dijkstra(self, start_node, end_node):
        visited = []
        if type(end_node) == tuple:
            end_node = self.nodes.index(end_node)
        D = {v: float('inf') for v in range(len(self.nodes))}
        W = {v: 'nan' for v in range(len(self.nodes))}
        D[self.nodes.index(start_node)] = 0

        pq = PriorityQueue()
        start_node_inx = self.nodes.index(start_node)
        pq.put((0, start_node_inx))

        while not pq.empty():
            (dist, current_vertex) = pq.get()
            visited.append(current_vertex)

            for neighbor in range(len(self.nodes)):
                if self.adjacency[current_vertex][neighbor] != -1:
                    distance = self.adjacency[current_vertex][neighbor]
                    if neighbor not in visited:
                        old_cost = D[neighbor]
                        new_cost = D[current_vertex] + distance
                        if new_cost < old_cost:
                            pq.put((new_cost, neighbor))
                            D[neighbor] = new_cost
                            W[neighbor] = current_vertex

        curr_node = W[end_node]
        path = [self.nodes[end_node]]
        if curr_node != self.nodes.index(start_node):
            if curr_node == "nan":
                return "inf", [start_node]
            path.append(self.nodes[curr_node])
        while curr_node != self.nodes.index(start_node):
            curr_node = W[curr_node]
            if curr_node == self.nodes.index(start_node):
                break
            path.append(self.nodes[curr_node])
        return D[end_node], path[::-1]

    def step(self):
        """Advance the model by one step."""
        self.schedule.step()
        self.adjacency = [[-1 for i in range(len(self.nodes))] for j in range(len(self.nodes))]
        self.define_adjacency()
        at_dest_list = []
        for agent in self.agents:
            at_dest_list.append(agent.at_destination)
        if all(at_dest_list):
            self.running = False
        self.datacollector.collect(self)


def agent_portrayal(agent):
    portrayal = {"Filled": "true",
                 "Layer": 0}
    if agent.type == "Agent":
        portrayal["Shape"] = "circle"
        portrayal["Color"] = agent.display_color
        portrayal["r"] = 0.6
    if agent.type == "Obstacle":
        portrayal["Shape"] = "rect"
        portrayal["Color"] = "black"
        portrayal["w"] = 0.7
        portrayal["h"] = 0.7
    if agent.type == "Target":
        portrayal["Shape"] = "arrowHead"
        portrayal["Color"] = agent.master_agent.display_color
        portrayal["scale"] = 0.4
        portrayal["Layer"] = 1
        portrayal["heading_x"] = 0
        portrayal["heading_y"] = -1

    return portrayal


height_width_sl = mesa.visualization.UserSettableParameter('slider', "Taille de la grille", 10, 10, 15, 1)
num_agents_sl = mesa.visualization.UserSettableParameter('slider', "Nombre d'agents", 5, 1, 10, 1)
num_obstacles_sl = mesa.visualization.UserSettableParameter('slider', "Nombre d'obstacles", 1, 1, 10, 1)
algo_type_sl = mesa.visualization.UserSettableParameter('slider', "Type d'algo (0- A*, 1- A* Coopératif, 2- A* + StochasticLocalSearch)", 2, 0, 2, 1)

#chart = mesa.visualization.ChartModule([{"Label": "TOTAL",
#                      "Color": "Red"}],
#                    data_collector_name='datacollector')

grid = mesa.visualization.CanvasGrid(agent_portrayal, 10, 10, 500, 500)
server = mesa.visualization.ModularServer(
    PathModel, [grid], "Model", {"width": height_width_sl,
                                        "num_agents": num_agents_sl,
                                        "num_obstacles": num_obstacles_sl,
                                        "algo_type": algo_type_sl}
)
server.port = 8521  # The default
server.launch()
