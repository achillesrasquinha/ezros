#!/usr/bin/env python

import ezros

node = ezros.Node("test")

@node.pub("/test", rate = 10, queue_size = 10)
def handle_test(message):
    message.data = "pong"

    return message

node.run()