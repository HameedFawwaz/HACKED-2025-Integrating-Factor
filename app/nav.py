import numpy as np


class Data:
    def __init__(self):
        self.data = {"acc": [], "vel": [], "pos": [], "omega": [], "theta": []}
        

    
    def update_data(self, acc: tuple, vel: tuple, pos: tuple, w: tuple, theta: tuple):
        self.data["acc"].append(acc)
        self.data["vel"].append(vel)
        self.data["pos"].append(pos)
        self.data["theta"].append(theta)
        self.data["omega"].append(w)
        