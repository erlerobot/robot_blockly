import os
import sys
import math
import socket
import rosgraph
import rospy

_caller_apis = {}
def get_api(master, caller_id):
    """
    Get XML-RPC API of node
    :param master: XML-RPC handle to ROS Master, :class:`xmlrpclib.ServerProxy`
    :param caller_id: node name, ``str``
    :returns: XML-RPC URI of node, ``str``
    :raises: :exc:`ROSTopicIOException` If unable to communicate with master
    """
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")
        except rosgraph.MasterError:
            caller_api = 'unknown address %s'%caller_id

    return caller_api

class ROSTopicException(Exception):
    """
    Base exception class of rostopic-related errors
    """
    pass

def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val


def get_info_text(topic):
    """
    Get human-readable topic description
    
    :param topic: topic name, ``str``
    """
    try:
        from cStringIO import StringIO
    except ImportError:
        from io import StringIO
    import itertools
    buff = StringIO()
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master('/rostopic')
    try:
        state = master.getSystemState()

        pubs, subs, _ = state
        # filter based on topic
        subs = [x for x in subs if x[0] == topic]
        pubs = [x for x in pubs if x[0] == topic]

        topic_types = _master_get_topic_types(master)
            
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    if not pubs and not subs:
        raise ROSTopicException("Unknown topic %s"%topic)

    buff.write("Type: %s\n\n"%topic_type(topic, topic_types))

    if pubs:
        buff.write("Publishers: \n")
        for p in itertools.chain(*[l for x, l in pubs]):
            buff.write(" * %s (%s)\n"%(p, get_api(master, p)))
    else:
        buff.write("Publishers: None\n")
    buff.write('\n')

    if subs:
        buff.write("Subscribers: \n")
        for p in itertools.chain(*[l for x, l in subs]):
            buff.write(" * %s (%s)\n"%(p, get_api(master, p)))
    else:
        buff.write("Subscribers: None\n")
    buff.write('\n')
    return [buff.getvalue(), topic_type(topic, topic_types)]

def rostopic_info(topic):
    """
    Print topic information to screen.
    
    :param topic: topic name, ``str``
    """
    print(get_info_text(topic)[0])
    print(get_info_text(topic)[1])


def info_node():
    rostopic_info("/usb_cam/image_raw/compressed")

if __name__ == "__main__":    
    info_node()

