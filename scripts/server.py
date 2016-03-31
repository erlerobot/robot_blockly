###############################################################################
#
# The MIT License (MIT)
#
# Copyright (c) Tavendo GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
###############################################################################

from autobahn.asyncio.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory

import json
import os
import sys
import math

import rosgraph.impl.graph
import rospy

import ast

import rostopicinfo

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
    return nodes


def rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    nodes_vector = _sub_rosnode_listnodes(namespace=namespace, list_uri=list_uri, list_all=list_all);
    return nodes_vector

class MyServerProtocol(WebSocketServerProtocol):

    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))

    def onOpen(self):
        print("WebSocket connection open.")

    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))
#            parser_json = json.loads("{0}".format(payload.decode('utf8')))
        command = ""
        try:
            parser_json = ast.literal_eval(json.loads(payload.decode('utf8')))
            # json.loads( json.dumps(payload.decode('utf8'), separators=(',', ':'), sort_keys=True) ) 
            commands = parser_json['command']
            command = commands[0]
            print(commands[0])
        except Exception as inst:
            print(inst.args) 

        if(command =='get_nodes'):

            _graph = rosgraph.impl.graph.Graph()
            _graph.set_master_stale(5.0)
            _graph.set_node_stale(5.0)
            _graph.update()
            json_to_send= {}
            json_to_send['nodes'] = list(_graph.nn_nodes)
            index = 0;
            Edges_json = {}
            for edges in _graph.nn_edges.edges_by_end:
                for edge in _graph.nn_edges.edges_by_end[edges]:
                    Edges_json[index] = {"label": edge.label, "start": edge.start, "end": edge.end }
                    index = index + 1
            print(Edges_json)
            json_to_send['edges'] = Edges_json
            json_to_send = json.dumps(json_to_send)

            namespace = None
            self.sendMessage(json_to_send, isBinary)
        #Pasear Topicos
        elif command=='topic': 
            print(rostopicinfo.get_info_text(commands[1])[1])
            json_to_send={}
            json_to_send['msg'] = rostopicinfo.get_info_text(commands[1])[1]
            self.sendMessage(json.dumps(json_to_send), isBinary)
        elif command=='node':
            print("node")
        else:
            self.sendMessage('Error', isBinary)

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))


if __name__ == '__main__':

    try:
        import asyncio
    except ImportError:
        # Trollius >= 0.3 was renamed
        import trollius as asyncio

    # factory = WebSocketServerFactory(u"ws://127.0.0.1:9000", debug=False)
    factory = WebSocketServerFactory(u"ws://0.0.0.0:9000")
    factory.protocol = MyServerProtocol

    loop = asyncio.get_event_loop()
    coro = loop.create_server(factory, '0.0.0.0', 9000)
    server = loop.run_until_complete(coro)

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        loop.close()
