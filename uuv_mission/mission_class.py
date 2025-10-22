import pandas as pd

class Mission:
    # Constructor to initialize a Mission object
    def __init__(self, reference, cave_height, cave_depth):
        self.reference = reference
        self.cave_height = cave_height
        self.cave_depth = cave_depth

    @classmethod
    def from_csv(cls, filepath):
        # Class method to create a Mission object from a CSV file.
        df = pd.read_csv(filepath)
        required_cols = {"reference", "cave_height", "cave_depth"}
        
        # Error check theres the right number of columns
        if not required_cols.issubset(df.columns):
            raise ValueError(f"CSV must contain columns: {required_cols}")
        # Create a Mission instance using the columns from the DataFrame
        return cls(
            reference=df["reference"],
            cave_height=df["cave_height"],
            cave_depth=df["cave_depth"]
        )

    def __repr__(self):
        # String representation showing how many entries are in the reference for troubleshooting
        return f"<Mission with {len(self.reference)} entries>"