# rosutils
> ROS interfacing made simple.

**rosutils** is a simple yet effective library to interface with ROS.

### Quickstart

```python
# import rosutils
import rosutils

# create a node...
node = rosutils.Node("my_awesome_node")

# subscribe to a topic...
# set message type using the parameter mtype
@node.sub("/my_awesome_topic", mtype = "geometry_msgs.msg.Vector3")
def handle_my_awesome_topic(data):
    # do something with data...
    pass

# handle a publisher...
# set the rate in "Hz" and the queue size.
@node.pub("/my_publish_topic", mtype = "geometry_msgs.msg.Twist", rate = 10, queue_size = 10)
def handle_my_publish_topic(message):
    factor = 10

    # increase by a factor of 10
    message.linear.x *= factor
    message.linear.y *= factor

    # return the message to have it published.
    return message
```