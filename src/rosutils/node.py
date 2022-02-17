from contextlib import contextmanager
import rospy
from std_msgs.msg import (
    String
)
from geometry_msgs.msg import (
    Twist
)

from bpyutils.util.string import get_random_str

MESSAGE_TYPES = {
    "string": {
        "type": String
    },
    "twist": {
        "type": Twist
    }
}

DEFAULT = {
    "rate": 10, # Hz
    "mtype": "string",
    "queue_size": 10
}

def get_message_conf(mtype):
    if mtype is None:
        config = MESSAGE_TYPES.get(mtype, "string")

    if isinstance(mtype, str):
        if mtype not in MESSAGE_TYPES:
            raise ValueError("No message type %s found." % mtype)
        else:
            config = MESSAGE_TYPES[mtype]
    else:
        config = { "type": mtype }    
    
    return config

class Node:
    def __init__(self, name = None, *args, **kwargs):
        self._name  = name  or "N%s" % get_random_str()

        anonymous   = kwargs.get("anonymous", True)

        rospy.init_node(name, anonymous = anonymous)

        rospy.loginfo("Initialized Node: %s." % name)

    def on(self, topic, mtype = "string"):
        message_conf = get_message_conf(mtype)
        message_type = message_conf["type"]

        def decorator(handler):
            self._subscriber = rospy.Subscriber(topic, message_type, handler)

        return decorator

    @contextmanager
    def pub(self, topic, mtype = DEFAULT["mtype"],
        rate       = DEFAULT["rate"],
        queue_size = DEFAULT["queue_size"]
    ):
        message_conf = get_message_conf(mtype)
        message_type = message_conf["type"]

        rospy_rate   = rospy.Rate(rate)

        def decorator(handler):
            def wrapper(*args, **kwargs):
                self._publisher = rospy.Publisher(topic, message_type, queue_size = queue_size)

                while not rospy.is_shutdown():
                    message = message_type()

                    message = handler(message)

                    self._publisher.publish(message)

                    rospy_rate.sleep()

            return wrapper

        return decorator

    def run(self):
        rospy.spin()