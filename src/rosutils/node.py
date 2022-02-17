import rospy
from std_msgs.msg import (
    String
)
from geometry_msgs.msg import (
    Twist
)

from bpyutils.util.string import get_random_str, safe_encode # , safe_decode
from bpyutils._compat     import iteritems

import serial

def safe_decode(obj, encoding = "utf-8"):
    try:
        obj = obj.decode(encoding)
    except (AttributeError, UnicodeDecodeError):
        pass
    
    return obj

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

        self._publishers = { }

    def on(self, topic, mtype = "string"):
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
                mtype = mtype, rate = rate, queue_size = queue_size)

        return decorator

    def _create_publisher_conf(self, topic,
        mtype        = DEFAULT["mtype"],
        rate         = DEFAULT["rate"],
        queue_size   = DEFAULT["queue_size"],
        handler      = lambda x: x
    ):
        message_conf = get_message_conf(mtype)
        message_type = message_conf["type"]

        return {
            "publisher": rospy.Publisher(topic, message_type, queue_size = queue_size),
            "rate": rate,
            "callback": handler,
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

class SerialNode(Node):
    def __init__(self, *args, **kwargs):
        port = kwargs["port"]
        baud = kwargs.get("baudrate", 9700)

        protocol = kwargs.get("protocol")
        
        self._super = super(SerialNode, self)
        self._super.__init__(*args, **kwargs)

        self.serial = serial.Serial(port, baud)

        self._protocol = protocol

    def get_serial_packet(self):
        return self._protocol.read_packet(self)

    def safe_serial_read(self):
        bytes_  = self.serial.read()
        decoded = safe_decode(bytes_)

        return decoded

    def safe_serial_write(self, data):
        encoded = safe_encode(data)
        self.serial.write(encoded)

    def write_serial_packet(self, data):
        self._protocol.write_packet(self, data)