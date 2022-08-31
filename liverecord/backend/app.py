from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO, send, emit

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*')
CORS(app)

@app.route("/")
def hello_world():
    return "TEST"

@socketio.on('connect')
def testing(data):
    print("Are we running?")
    emit("Testing", "Hello there!")

@socketio.on('connect_error')
def what_went_wrong(data):
    print("Something went wrong")
    print(data)
    
if __name__ == '__main__':
    socketio.run(app)