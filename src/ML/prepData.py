from typing import TUple, List

import numpy as np
import pandas as pd
from sklearn.preprocessing import LabelEncoder

TimeSteps = 30
NumJoints = 17
NumCoords = 3

MOVEMENT_COL = "movement_class"
SCORE_COL = "deviation_score"
ERROR_COL = "error_type"

def justCoordinateColumns (df: pd.DataFrame) -> List[str]: #theoretically the columns for what the coords are are in the back. IDK 

    theLabel_col = {MOVEMENT_COL, SCORE_COL, ERROR_COL}
    justCoords_col = [ c for c in df.columns if c not in theLabel_col]
    return justCoords_col 


def reshape_row_to_sequence(row: pd.Series, coord_cols: List[str]) -> np.ndarray:
    """
    Converts one row of flattened coordinates into shape (30, 17, 3).
    """
    values = row[coord_cols].to_numpy(dtype=np.float32)
    seq = values.reshape(TimeSteps,NumCoords,NumJoints)
    return seq


