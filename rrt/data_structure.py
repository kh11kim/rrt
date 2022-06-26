import numpy as np
from typing import Optional
from dataclasses import dataclass, field

@dataclass
class Config:
    q: np.ndarray
    index: int = field(default_factory=lambda :-1)

    def copy(self)->"Config":
        return Config(self.q)
    

class Tree:
    def __init__(self, root: Config):
        root.index = 0
        self.root = root
        self.data = [root]
        self.parent = {0:-1}
        self.num = 1

    def add_node(self, node: Config, parent: Config):
        assert node.index != parent.index
        node.index = self.num
        self.parent[node.index] = parent.index
        self.data.append(node)
        self.num += 1
    
    def nearest(self, node: Config):
        distances = []
        for node_tree in self.data:
            d = np.linalg.norm(node_tree.q - node.q)
            distances.append(d)
        idx = np.argmin(distances)
        return self.data[idx]
    
    def backtrack(self, node: Config):
        path = []
        node_curr = node
        while True:
            path.append(node_curr)
            parent_index = self.parent[node_curr.index]
            if parent_index == -1:
                break
            node_curr = self.data[parent_index]
        return path[::-1]
