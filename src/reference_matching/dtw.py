from darts import TimeSeries, set_option, dtw
from pandas import read_csv, DataFrame, Series
from typing import NamedTuple
from matplotlib import pyplot as plt

set_option("plotting.use_darts_style", True)

class reference(NamedTuple):
    workout_name: str
    csv_name: str

def get_reference_list() -> list[reference] | list[None]:
    reference_csv = read_csv("/references/list.csv")
    
    return [reference(row["workout"], row["file"]) for _, row in reference_csv.iterrows()]

def read_workout_reference(filename: str) -> DataFrame | None:
    try:
        csv = read_csv(filename)
    except:
        print("could not read file")
        return None
    
    return csv

def match_reference(reference: TimeSeries, captured: TimeSeries) -> TimeSeries:
    multigrid_radius = 10
    
    