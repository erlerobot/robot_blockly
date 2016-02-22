import os
import sys
import math

import rosgraph
import rospy

def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val

def _sub_rostopic_list(master, pubs, subs, publishers_only, subscribers_only, verbose, indent=''):
    def topic_type(t, topic_types):
        matches = [t_type for t_name, t_type in topic_types if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    if verbose:
        topic_types = _master_get_topic_types(master)

        if not subscribers_only:
            print("\n%sPublished topics:"%indent)
            for t, l in pubs:
                if len(l) > 1:
                    print(indent+" * %s [%s] %s publishers"%(t, topic_type(t, topic_types), len(l)))
                else:
                    print(indent+" * %s [%s] 1 publisher"%(t, topic_type(t, topic_types)))                    

        if not publishers_only:
            print(indent)
            print(indent+"Subscribed topics:")
            for t,l in subs:
                if len(l) > 1:
                    print(indent+" * %s [%s] %s subscribers"%(t, topic_type(t, topic_types), len(l)))
                else:
                    print(indent+" * %s [%s] 1 subscriber"%(t, topic_type(t, topic_types)))
        print('')
    else:
        if publishers_only:
            topics = [t for t,_ in pubs]
        elif subscribers_only:
            topics = [t for t,_ in subs]
        else:
            topics = list(set([t for t,_ in pubs] + [t for t,_ in subs]))
        topics.sort()
        print('\n'.join(["%s%s"%(indent, t) for t in topics]))

def _rostopic_list(topic, verbose=False,
                   subscribers_only=False, publishers_only=False,
                   group_by_host=False):
    master = rosgraph.Master('/rostopic')
    try:
        state = master.getSystemState()

        pubs, subs, _ = state
        if topic:
            # filter based on topic
            topic_ns = rosgraph.names.make_global_ns(topic)        
            subs = (x for x in subs if x[0] == topic or x[0].startswith(topic_ns))
            pubs = (x for x in pubs if x[0] == topic or x[0].startswith(topic_ns)) 
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    _sub_rostopic_list(master, pubs, subs,
                       False, False,
                       True)

def main():
    topic =None
    #topic = rosgraph.names.script_resolve_name('rostopic', 'rostopic')
    exitval = _rostopic_list(topic,
                             verbose=True,
                             subscribers_only=False,
                             publishers_only=False,
                             group_by_host=False) or 0
    if exitval != 0:
    	sys.exit(exitval)

if __name__ == "__main__":
    # filter out remapping arguments in case we are being invoked via roslaunch
    argv = rospy.myargv(sys.argv)
    #rospy.init_node('Rostopiclist', anonymous=True)
    
    main()