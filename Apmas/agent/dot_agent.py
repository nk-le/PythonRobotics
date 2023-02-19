from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt


class DotAgent:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def move(self, dx, dy):
        self.x = self.x + dx 
        self.y = self.y + dy 

    def get(self):
        return [self.x, self.y]