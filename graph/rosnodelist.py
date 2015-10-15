import os
import sys
import math

import rosgraph
import rospy

ID = '/rosnode'
def get_node_names(namespace=None):

    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    nodes = []
    if namespace:
        # canonicalize namespace with leading/trailing slash
        g_ns = rosgraph.names.make_global_ns(namespace)
        for s in state:
            for t, l in s:
                nodes.extend([n for n in l if n.startswith(g_ns) or n == namespace])
    else:
        for s in state:
            for t, l in s:
                nodes.extend(l)
    return list(set(nodes))


def _sub_rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    master = rosgraph.Master(ID)
    nodes = get_node_names(namespace)
    nodes.sort()
    if list_all:
        return '\n'.join(["%s \t%s"%(get_api_uri(master, n) or 'unknown address', n) for n in nodes])
    elif list_uri:
        return '\n'.join([(get_api_uri(master, n) or 'unknown address') for n in nodes])
    else:
        return '\n'.join(nodes)
    

def rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    print(_sub_rosnode_listnodes(namespace=namespace, list_uri=list_uri, list_all=list_all))
    

def list_nodes():
    namespace = None

    rosnode_listnodes(namespace=namespace, list_uri=False, list_all=False)

if __name__ == "__main__":    
    list_nodes()