# imports - import ros packages
import rospy

from bpyutils.util.string  import get_random_str
from bpyutils.util.imports import import_handler
from bpyutils.log import get_logger

def safe_decode(obj, encoding = "utf-8"):
    try:
        obj = obj.decode(encoding)
    except (AttributeError, UnicodeDecodeError):
        pass
    
    return obj

MESSAGE_TYPES = {
    None: {
        "type": "std_msgs.msg.Empty",
    },
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

def get_mtype(message):
    if isinstance(message, str):
        return "string"

def get_message_conf(mtype):
    config = MESSAGE_TYPES.get(mtype, "string")

    if isinstance(mtype, str):
        if "." in mtype:
            mtype  = import_handler(mtype)
            config = { "type": mtype }
        elif mtype not in MESSAGE_TYPES:
            raise ValueError("No message type %s found." % mtype)
        else:
            config = { "type": import_handler(config["type"]) }
    else:
        if mtype is None:
            mtype = import_handler(config["type"])
        elif "type" in mtype and isinstance(mtype["type"], str):
            mtype = import_handler(mtype["type"])
            
        config = { "type": mtype }
    
    return config

class Node:
    def __init__(self, name = None, *args, **kwargs):
        self._name   = name or "N%s" % get_random_str()

        anonymous    = kwargs.get("anonymous", True)

        self._logger = get_logger("Node %s" % self._name)

        rospy.init_node(name, anonymous = anonymous)

        self.log("Initialized Node: %s." % name)

        self._publishers  = { }
        self._subscribers = { }

    @property
    def name(self):
        return getattr(self, "_name", None)

    def on(self, topic, mtype = "string"):
        self.log("Subscribing to topic: %s (type %s)..." % (topic, mtype))

        message_conf = get_message_conf(mtype)
        message_type = message_conf["type"]

        def decorator(handler):
            self._subscribers[topic] = rospy.Subscriber(topic, message_type, handler)

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
                self._publishers[topic] = self._create_publisher_conf(topic, mtype = get_mtype(message), *args, **kwargs)
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

                    rate         = self.get_param("rate", config["rate"])
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

    # log with prefixing [node name] "log string"
    # easier to debug logs from different nodes.
    def log(self, string):
        """
            Log information.
            Example
                >>> node.log("This is a log string.")
        """
        # build the log string
        name   = self.name
        string = "[%s] %s" % (name, string) # "[node name] log str"
        
        # you can control the logging output by just providing this
        # flag as a parameter. 
        # verbose if true, will eventually log it.
        # note, the --screen flag passed within roslaunch is still
        # considered over this kind of verbosity.
        verbose = rospy.get_param("verbose", True)

        if verbose:
            # log it using rospy.loginfo
            self._logger.info(string)

    # a helper function to build a parameter name
    def _create_param_name(self, name):
        """
            A helper function to build a parameter name for a given node.
            Example
                >>> node = MyNode(name = "foobar")
                >>> node._create_param_name("rate")
                "foobar/rate"
        """
        param_name = self.name + "/" + name # concatenate the name/param_name
        return param_name
    
    def get_param(self, name, default = None, **kwargs):
        """
            A get parameter helper. Fetches a parameter defined for a node.
            Example
                >>> node = MyNode(name = "foobar")
                >>> node.get_param("foobar")
                "foobar_value"
        """
        # create a parameter name
        param_name = self._create_param_name(name)
        # get the parameter using rospy.get_param
        return rospy.get_param(param_name, default, **kwargs)

    def set_param(self, name, value, **kwargs):
        """
            A set parameter helper. Creates a parameter defined for a node.
            Example
                >>> node = MyNode(name = "foobar")
                >>> node.set_param("foobar", "foobar_value") # value set for foobar/foobar
        """
        # create a parameter name
        param_name = self._create_param_name(name)
        # set the parameter
        rospy.set_param(param_name, value, **kwargs)

    def p(self, name, default = None, **kwargs):
        return self.get_param(name, default = default, **kwargs)