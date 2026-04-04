from typing import Tuple, List

import numpy as np
import pandas as pd
from sklearn.preprocessing import LabelEncoder

CSV = "/Users/erichuang/Documents/dev/CMPE 195 Project/group-project-random-strangers/src/ML/aerobics_skeleton_dataset.csv" #my filepath, replace with yours
df = pd.read_csv (CSV)
TimeSteps = 30. #taken from the kaggle page
NumJoints = 17
NumCoords = 3

MOVEMENT_COL = "movement_class"
SCORE_COL = "deviation_score"
ERROR_COL = "error_type"

def justCoordinateColumns (df: pd.DataFrame) -> List[str]: #i hope the labels are just in the back for stuff like error percentage

    theLabel_col = {MOVEMENT_COL, SCORE_COL, ERROR_COL}
    justCoords_col = [ c for c in df.columns if c not in theLabel_col]
    return justCoords_col 


def reshape_row_to_sequence(row: pd.Series, justCoord_cols: List[str]) -> np.ndarray: 
  
   # Converts one row of flattened coordinates into shape (30, 17, 3). Thank AI cause i actually dont know how to properly write this func

    values = row[justCoord_cols].to_numpy(dtype=np.float32)
    seq = values.reshape(TimeSteps,NumCoords,NumJoints)
    return seq

def dataNormalizer9000 ( #need to normalize data so that people standing randomly don't get judged for being off center
        seq: np.ndarray,
        #making assumptions for where joints are, need to change these maybe
        torso_idx: int = 0,
        left_shoulder_idx: int = 5,
        right_shoulder_idx: int = 6,

) -> np.ndarray:
    
    seqCopy = seq.copy() #make a copy so I can reaccess the original later
    Torso = seqCopy[:, torso_idx:torso_idx + 1] #basically, for every frame of time (30), take the torso joint and xyz of said pelvis

    seqCopy = seqCopy - Torso #make the torso 000 in the frame. Torso is the new refernce peoint for everything else (the two shoulders lmao)


    shoulderVector = seqCopy [:, left_shoulder_idx, :] - seqCopy [:, right_shoulder_idx, :] #vector of left shoudler to right one. Shape is [30,3]
    shoulderDistance = np.linalg.norm(shoulderVector,axis=1) #shape 30,1

    seqCopy = seqCopy  / shoulderDistance[:, None, :] #divides each frame by shoulder distance, which should make larger and smaller skeletons comparable
    
    return seqCopy

def datasetBuild (CSV: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, LabelEncoder, LabelEncoder]: 
#X, y score, y movement, y error, movement _encoder, errorenc

#X - each CSV row and its data
#y score - deviation from the correct
# y movement what movement? lunge, sidestep, etc 
#encoders turn numbers into labels. ie lunge sidestep lunge is 0,1,0, but we cant read that, so encoder

    requiredColumns = [MOVEMENT_COL,SCORE_COL, ERROR_COL]  #not sure we are missing data but its good to check i guess
    for col in requiredColumns:
        if col not in df.columns:
            raise ValueError (f"Missing Data in column: {col}")
        
    coordinateColumns = justCoordinateColumns ( df) #gets the coordinate columns from the prior function and the dataframe

    Validated = TimeSteps * NumJoints * NumCoords #theoretically 1530, may need to replace when with live data
    if len(coordinateColumns) != Validated: 
        raise ValueError(
            f"Expect: {Validated} coords, found {len(coordinateColumns)}"
            f"coordinate columns found: {coordinateColumns[:10]}"
        )
    #lists that should hold data samples.

    sequences = []
    scores = []
    movements = []
    fuckups = []

    for _, row in df. iterrows(): #iterates through the CSV
        seq = reshape_row_to_sequence(row, coordinateColumns) #turn rows into 30 17 3 arrays
        seq = dataNormalizer9000

        sequences.append(seq)
        scores.append(str(row[SCORE_COL]))
        movements.append(str(row[MOVEMENT_COL]))
        fuckups.append(str(row[ERROR_COL]))

    X = np.stack(sequences).astype(np.float32) #combine all sequences into a large numpy array   
    yScore = np.array(scores, dtype=np.float32)
    movementencode = LabelEncoder() 
    errorEncode = LabelEncoder() 

    yMovement = movementencode.fit_transform(movements) #need to convert these strings into integers, i think
    yError = errorEncode.fit_transform(fuckups) #like [arm delay, misaligned knee, arm delay -> 0, 1, 0.] I have no idea how many possible errors there are, but theres a lot

    #also, we can return yError and yMovement, but i dont think its needed right now, since I am only building a regressor. With the full feedback system, this will become needed.

    return X, yScore, yMovement, movementencode,errorEncode

#AI tells me that 4d tensor array sare not popular with the general models, and was leading to my crappy predictions. Need to flatten the arraysdown to 2d. 

def goombaStomp (X: np.ndarray) -> np.ndarray: #(sampleSize, 30, 17 ,3 ) -> (sampleSize, 1530)
    return X.reshape(X.shape[0],-1) 

def saveData( #stores everything into arrays
    output_path: str,
    X: np.ndarray,
    y_score: np.ndarray,
    y_movement: np.ndarray,
    y_error: np.ndarray,
):
    np.savez_compressed ( #save it onto an NPZ file for the next file
        output_path,
        X=X,
        y_score=y_score,
        y_movement=y_movement,
        y_error=y_error
    )
    print(f"Saved data to {output_path}")

if __name__ == "__main__": #only run this file if directly run. No need to repeatedly parse the CSV file if nothing changed, i think
    out_path = "processed_data.npz"
    X, y_score, y_movement, y_error, movement_enc, error_enc = datasetBuild(CSV)

    goombaX = goombaStomp(X)
    print("Flattened Shape:", goombaX.shape)

    saveData(out_path, X, y_score, y_movement, y_error)

    #with the data flattened, normalized, and parsed, next is to train the model. 