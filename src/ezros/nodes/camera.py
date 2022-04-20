#!/usr/bin/env python

import ezros
import cv2

node    = ezros.Node("ezros_camera")
capture = cv2.VideoCapture(0)

@node.pub("/camera", rate = 10, queue_size = 10)
def handle_test(message):
    message.data = "pong"

    return message

if __name__ == "__main__":
    node.run()