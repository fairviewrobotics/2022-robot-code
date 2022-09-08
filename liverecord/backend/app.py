from random import random
from typing import Dict, List
from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO, send, emit
from networktables import NetworkTables

NetworkTables.initialize()
table = NetworkTables.getTable("liverecord")
values: Dict[str, List[List[float]]] = {
    "pid1": [
        [0.1, 1],
        [0.2, 2],
        [0.3, 3]
    ]
}

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')
CORS(app)


def entryListener(source, key, value: str, isNew):
    print("RUNNING")
    print(f"{key} | {value}")
    newRecord = value.split(",")

    if key in values:
        values[key].append(newRecord)
    else:
        values[key] = []
        values[key].append(newRecord)
    
    package = {}
    package["key"] = key
    package["value"] = newRecord
    socketio.emit("newValue", package) 

table.addEntryListener(entryListener)

@app.route("/")
def hello_world():
    return "TEST"

@socketio.on('connect')
def testing(data):
    print(values)
    emit("fullPayload", values)

if __name__ == '__main__':
    socketio.run(app)