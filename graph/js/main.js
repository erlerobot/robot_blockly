var socket = null;
var isopen = false;

var ros_graph = new ROS_Graph( {width: 1160, height: 600})

function sendText() {
	if (isopen) {
		socket.send("\"{'command':['get_nodes']}\"");
		console.log("Text message sent.");               
	} else {
		console.log("Connection not opened.")
	}
};

window.onload = function() {

    socket = new WebSocket("ws://127.0.0.1:9000");
    socket.binaryType = "arraybuffer";

    socket.onopen = function() {
       console.log("Connected!");
       isopen = true;
    }

    socket.onmessage = function(e) {
        if (typeof e.data == "string") {
			console.log("Text message received: " + e.data);
			json_revieved = JSON.parse(e.data)
			if(json_revieved['msg']!='undefined'){
				if(json_revieved['msg']=='sensor_msgs/Image' || json_revieved['msg']=='sensor_msgs/CompressedImage'  ){
					ros_graph.set_camera_topic("lol");
				}
			}
			if(json_revieved['edges'] != undefined){
////////////////////////////////////////////////////////////
	        	nodesList = ros_graph.get_nodes(e);
	        	linksList = ros_graph.get_links(e, nodesList);
	        	ros_graph.draw_graph();
	    	}
////////////////////////////////////////////////////////////
	    }
	}
	socket.onclose = function(e) {
		console.log("Connection closed.");
		socket = null;
		isopen = false;
	}
};