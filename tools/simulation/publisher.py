from gz.msgs11.twist_pb2 import Twist
from gz.transport14 import Node
import time
import math

from dataclasses import dataclass, field

@dataclass
class Command:
    linear:list = field(default_factory=lambda: [0,0,0])
    angular: float=0
    delay: float =0


def main():
    node = Node()

    # CONFIGURATION ---------------------------------------------------------

    topic = f"/quadcopter/vel_cmd"
    # -----------------------------------------------------------------------

    pub_wrench = node.advertise(topic, Twist)

    print(f"Publishing forces to: {topic}")
    print("Press Ctrl+C to stop...")

    # Hover variables
    # It 
    cmds = [Command([0,0,1]),
            Command([0,0,0],delay=5),
            Command([1,0,0],delay=5)
            ]
    time.sleep(1)
    try:
        for cmd in cmds:
            time.sleep(cmd.delay)
            # Create the message
            msg = Twist()
            # msg.linear.FromString('1,2,3')
            msg.linear.x=cmd.linear[0]
            msg.linear.y=cmd.linear[1]
            msg.linear.z=cmd.linear[2]
            msg.angular.z=cmd.angular
            # Publish
            pub_wrench.publish(msg)
        

    except KeyboardInterrupt:
        print("\nStopping force test.")

if __name__ == "__main__":
    main()