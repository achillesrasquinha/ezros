# rosutils

### Quickstart

```python
import rosutils

node = rosutils.Node()

@node.sub("/my_awesome_topic")
def handle_my_awesome_topic(data):
    # do something with data...
    pass

@node.pub("/my_publish_topic", mtype = "twist", rate = 10, queue_size = 10)
def handle_my_publish_topic(message):
    factor = 10

    # increase by a factor of 10
    message.linear.x *= factor
    message.linear.y *= factor

    return message
```