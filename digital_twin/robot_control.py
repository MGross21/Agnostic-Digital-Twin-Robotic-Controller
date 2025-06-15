from armctl import *
from communication.localpubsub import LocalPubSub

### Object Z HEIGHTS ###
# Table: + 150 mm
# Inside Cup: + 165 mm

### Robot Offset ###
# X: - 165mm
# Y: + 520.7 mm

with (
    Pro600("192.168.1.159", 5_001) as robot,
    # LocalPubSub(port=5_002) as pubsub,
):
    robot.home()