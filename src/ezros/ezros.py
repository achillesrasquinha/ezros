from __future__ import absolute_import

import rospy

def on_shutdown(decorator):
    def decorator(*args, **kwargs):
        return rospy.on_shutdown(*args, **kwargs)
    return decorator