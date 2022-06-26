import numpy as np
from typing import Callable, Optional
from rrt.data_structure import Config, Tree

class BiRRT:
    def __init__(
        self,
        is_col_fn: Callable[[Config], bool],
        rand_config_fn: Callable[[], np.ndarray],
        eps: float = 0.1,
        p_goal: float = 0.2,
        max_iter: int = 100,
        q_delta_max: float = 0.1,
        DLS_damping: float = 0.1,
    ):
        self.eps = eps
        self.p_goal = p_goal
        self.max_iter = max_iter
        self.q_delta_max = q_delta_max
        self.DLS_damping = DLS_damping

        self.is_collision = is_col_fn
        self.get_random_node = lambda : Config(rand_config_fn())
        self.is_goal = lambda node: self.distance(node, self.goal) < self.eps

    def plan(self, start: Config, goal: Config):
        self.init(start, goal)
        
        tree_a = self.tree_start
        tree_b = self.tree_goal
        for i in range(self.max_iter):
            node_rand = self.get_random_node()
            
            if not self.extend(tree_a, node_rand) == "trapped":
                if self.connect(tree_b, self._node_new) == "reached":
                    return self.get_path()
            (tree_a, tree_b) = (tree_b, tree_a)
        return []

    def init(self, start: Config, goal: Config):
        self.start = start
        self.goal = goal
        self.tree_start = Tree(start)
        self.tree_goal = Tree(goal)

    def connect(self, tree, node):
        result = "advanced"
        while result == "advanced":
            result = self.extend(tree, node)
        return result

    def distance(self, node1:Config, node2:Config):
        return np.linalg.norm(node1.q - node2.q)

    def extend(self, tree: Tree, node_rand: Config):
        node_near = tree.nearest(node_rand)
        node_new = self.control(node_near, node_rand)
        if node_new is not None:
            tree.add_node(node_new, node_near)
            if not self.distance(node_rand, node_new) > self.eps:
                self.last_node = node_new
                return "reached"
            else:
                self._node_new = node_new #save this to "connect"
            return "advanced"
        return "trapped"
    
    def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
        if q_delta_max is None:
            q_delta_max = self.eps
        mag = np.linalg.norm(q_delta) #2norm
        if mag > q_delta_max:
            q_delta = q_delta / mag * q_delta_max
        return q_delta

    def control(self, node_near:Config, node_rand:Config):
        mag = self.distance(node_near, node_rand)
        if mag <= self.eps:
            node_new = node_rand.copy()
        else:
            q_err = node_rand.q - node_near.q
            q_delta = self.limit_step_size(q_err)
            q_new = node_near.q + q_delta
            node_new = Config(q_new)

        if not self.is_collision(node_new):
            return node_new
        else:
            return None
    
    def get_path(self):
        node_tree_start = self.tree_start.nearest(self.last_node)
        node_tree_goal = self.tree_goal.nearest(self.last_node)
        path_from_start = self.tree_start.backtrack(node_tree_start)
        path_from_goal = self.tree_goal.backtrack(node_tree_goal)
        
        return [*path_from_start, *path_from_goal[::-1]]
