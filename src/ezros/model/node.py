# imports - import ros packages
import rospy

from bpyutils.util.string  import get_random_str
from bpyutils.util.imports import import_handler

def safe_decode(obj, encoding = "utf-8"):
    try:
        obj = obj.decode(encoding)
    except (AttributeError, UnicodeDecodeError):
        pass
    
    return obj

MESSAGE_TYPES = {
    "string": {
        "type": "std_msgs.msg.String"
    },
    "twist": {
        "type": "geometry_msgs.msg.Twist"
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
        if "." in mtype:
            mtype  = import_handler(mtype)
            config = { "type": mtype }
        elif mtype not in MESSAGE_TYPES:
            raise ValueError("No message type %s found." % mtype)
        else:
            MESSAGE_TYPES[mtype]["type"] = import_handler(MESSAGE_TYPES[mtype]["type"])
            config = MESSAGE_TYPES[mtype]
    else:
        if "type" in mtype and isinstance(mtype["type"], str):
            mtype = import_handler(mtype["type"])
            
        config = { "type": mtype }
    
    return config

class Node:
    def __init__(self, name = None, *args, **kwargs):
        self._name  = name or "N%s" % get_random_str()

        anonymous   = kwargs.get("anonymous", True)

        rospy.init_node(name, anonymous = anonymous)

        rospy.loginfo("Initialized Node: %s." % name)

        self._publishers = { }

    def on(self, topic, mtype = "string"):
        rospy.loginfo("Subscribing to topic: %s..." % topic)

        message_conf = get_message_conf(mtype)
        message_type = message_conf["type"]

        def decorator(handler):
            self._subscriber = rospy.Subscriber(topic, message_type, handler)

        return decorator

    def pub(self, topic, mtype = DEFAULT["mtype"],
        rate       = DEFAULT["rate"],
        queue_size = DEFAULT["queue_size"]
    ):
        def decorator(handler):
            self._publishers[topic] = self._create_publisher_conf(topic,
                mtype = mtype, rate = rate, queue_size = queue_size, callback = handler)

        return decorator

    def _create_publisher_conf(self, topic,
        mtype        = DEFAULT["mtype"],
        rate         = DEFAULT["rate"],
        queue_size   = DEFAULT["queue_size"],
        callback     = lambda x: x
    ):
        message_conf = get_message_conf(mtype)
        message_type = message_conf["type"]

        return {
            "publisher": rospy.Publisher(topic, message_type, queue_size = queue_size),
            "rate": rate,
            "callback": callback,
            "message_type": message_type
        }

    def pub_msg(self, topic, message, *args, **kwargs):
        create = kwargs.pop("create", False)

        if topic not in self._publishers:
            if create:
                self._publishers[topic] = self._create_publisher_conf(topic, mtype = type(message), *args, **kwargs)
            else:
                raise ValueError("No publisher of topic type %s found." % topic)

        publisher_conf = self._publishers[topic]

        publisher = publisher_conf["publisher"]
        publisher.publish(message)

    def run(self):
        if self._publishers:
            while not rospy.is_shutdown():
                for topic in list(self._publishers):
                    config       = self._publishers[topic]

                    rate         = config["rate"]
                    callback     = config["callback"]
                    publisher    = config["publisher"]
                    message_type = config["message_type"]

                    rospy_rate   = rospy.Rate(rate)

                    m_instance   = message_type()

                    message      = callback(m_instance)
                    
                    publisher.publish(message)

                    rospy_rate.sleep()
        else:
            rospy.spin()