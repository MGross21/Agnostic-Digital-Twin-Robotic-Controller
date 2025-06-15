from armctl import *
from communication.localpubsub import LocalPubSub



with (
    Pro600("192.168.1.159", 5_001) as robot,
    # LocalPubSub(port=5_002) as pubsub,
):
    robot.move_cartesian([250,0, ..., 0, 200]) # Not finished