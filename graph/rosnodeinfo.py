import os
import sys
import math
import socket

import rosgraph
import rostopic
import rospy
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

# need for calling node APIs
def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val

class ROSNodeException(Exception):
    """
    rosnode base exception type
    """
    pass
class ROSNodeIOException(ROSNodeException):
    """
    Exceptions for communication-related (i/o) errors, generally due to Master or Node network communication issues.
    """
    pass

ID = '/rosnode'
def get_node_connection_info_description(node_api, master):
    #turn down timeout on socket library
    socket.setdefaulttimeout(5.0)
    node = ServerProxy(node_api)
    system_state = master.getSystemState()

    try:
        pid = _succeed(node.getPid(ID))
        buff = "Pid: %s\n"%pid
        #master_uri = _succeed(node.getMasterUri(ID))
        businfo = _succeed(node.getBusInfo(ID))
        if businfo:
            buff += "Connections:\n"
            for info in businfo:
                dest_id   = info[1]
                direction = info[2]
                transport = info[3]
                topic     = info[4]
                if len(info) > 5:
                    connected = info[5]
                else:
                    connected = True #backwards compatibility

                if connected:
                    buff += " * topic: %s\n"%topic

                    # older ros publisher implementations don't report a URI
                    buff += "    * to: %s\n"%lookup_uri(master, system_state, topic, dest_id)
                    if direction == 'i':
                        buff += "    * direction: inbound\n"
                    elif direction == 'o':
                        buff += "    * direction: outbound\n"
                    else:
                        buff += "    * direction: unknown\n"
                    buff += "    * transport: %s\n"%transport
                    return buff
    except socket.error:
        raise ROSNodeIOException("Communication with node[%s] failed!"%(node_api))
    return buff 
_caller_apis = {}
def get_api_uri(master, caller_id, skip_cache=False):   
    caller_api = _caller_apis.get(caller_id, None)
    if not caller_api or skip_cache:
        try:
            caller_api = master.lookupNode(caller_id)
            _caller_apis[caller_id] = caller_api
        except rosgraph.MasterError:
            return None
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")
    return caller_api

def lookup_uri(master, system_state, topic, uri):
    for l in system_state[0:2]:
        for entry in l:
            if entry[0] == topic:
                for n in entry[1]:
                    if rostopic.get_api(master, n) == uri:
                        return '%s (%s)' % (n, uri)
    return uri

def get_node_info_description(node_name):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master(ID)

    # go through the master system state first
    try:
        state = master.getSystemState()
        pub_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    pubs = [t for t, l in state[0] if node_name in l]
    subs = [t for t, l in state[1] if node_name in l]
    srvs = [t for t, l in state[2] if node_name in l]  

    buff = "Node [%s]"%node_name
    if pubs:
        buff += "\nPublications: \n"
        buff += '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in pubs]) + '\n'
    else:
        buff += "\nPublications: None\n"
    if subs:
        buff += "\nSubscriptions: \n"
        buff += '\n'.join([" * %s [%s]"%(l, topic_type(l, pub_topics)) for l in subs]) + '\n'
    else:
        buff += "\nSubscriptions: None\n"        
    if srvs:
        buff += "\nServices: \n"
        buff += '\n'.join([" * %s"%l for l in srvs]) + '\n'
    else:
        buff += "\nServices: None\n"
        
    return buff

    
def rosnode_info(node_name):
    def topic_type(t, pub_topics):
        matches = [t_type for t_name, t_type in pub_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    master = rosgraph.Master(ID)
    node_name = rosgraph.names.script_resolve_name('rosnode', node_name)

    print('-'*80)
    print(get_node_info_description(node_name))
        
    node_api = get_api_uri(master, node_name)
    """
    if not node_api:
        print("cannot contact [%s]: unknown node"%node_name, file=sys.stderr)
        return
    """
    print("\ncontacting node %s ..."%node_api)

    print(get_node_connection_info_description(node_api, master))

def info_node():
    rosnode_info("/rostopic_29841_1444226670483")

if __name__ == "__main__":    
    info_node()
