import pandas as pd

class Mission:
    def __init__(self, cave_height, cave_depth, reference):
        self.cave_height = cave_height
        self.cave_depth = cave_depth
        self.reference = reference

    @classmethod
    def from_csv(cls, filepath="mission.csv"):
        data = pd.read_csv(filepath)
        return cls(
            cave_height=data["cave_height"].values,
            cave_depth=data["cave_depth"].values,
            reference=data["reference"].values
        )
    
