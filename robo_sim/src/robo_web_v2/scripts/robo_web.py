import rospy
from std_msgs.msg import Int16
from flask import Flask, render_template, request
from threading import Thread

app = Flask(__name__)
pub = None

def initialize_ros_node():
    global pub
    rospy.init_node('web_interface_node', anonymous=True, disable_signals=True)
    pub = rospy.Publisher('mode_topic', Int16, queue_size=10)

Thread(target=initialize_ros_node).start()

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/send_command/<command>", methods=["POST"])
def send_command(command):
    global pub
    if pub is None:
        initialize_ros_node()

    # Map commands to their respective values
    command_mapping = {
        "forward": 1,
        "backward": 2,
        "right": 3,
        "left": 4,
        "stop": 5,
        "spinlift": 6,
        "spinright": 7,
        "digleftforward": 8,
        "digrightforward": 10,
        "digleftbackward": 9,
        "digrightbackward": 11,
        "up": 12,
        "down": 13
    }

    mode_value = command_mapping.get(command)
    if mode_value is not None:
        pub.publish(Int16(mode_value))
        rospy.loginfo(f"Published command: {command} with value: {mode_value}")
        return "Command sent successfully", 200
    else:
        return f"Unknown command: {command}", 400

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
