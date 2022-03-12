# wait for Pi to start
echo "Waiting 5 seconds..."
sleep 5

# start ball vision
python3 ball_vision/ball_vision.py &

# TODO: start high goal vision
