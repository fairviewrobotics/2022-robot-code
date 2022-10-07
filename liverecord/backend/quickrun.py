from datetime import datetime
from random import random
from typing import Dict, List

from networktables import NetworkTable, NetworkTableEntry, NetworkTables, Value

NetworkTables.initialize(server='roborio-2036-frc.local')
table = NetworkTables.getTable("pidtunedrive")
values: List[str] = [ "1", "2", "3"]

def entryListener(one: NetworkTable, two: str, three: NetworkTableEntry, four: Value, five: int):
    if three.key == "pidtunedrive":
        values.append(three.value)

table.addEntryListener(entryListener)
input("This is just to pause the thread. click enter when done!")

with open(f"result{datetime.now()}.txt", "x") as file:
    file.write(values)

print("done!")