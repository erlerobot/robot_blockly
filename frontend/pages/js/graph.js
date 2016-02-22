var ROS_Graph = function(data) {
    this.load(data);   
};

ROS_Graph.prototype = {
	_thisGraph: this,
	_link: null,
	_node: null,
	_width: 0,
	_height: 0,
	_bondSelected_image: null,
	_atomSelected: null,
	_bondSelected: null,

	load: function(data){
		_thisGraph = this;
		_thisGraph._height = data.height;
		_thisGraph._width = data.width;
	},

	get_nodes: function(e){
	    json_revieved = JSON.parse(e.data)

		var nodesList = [];
		for(i = 0; i < json_revieved['nodes'].length; i++){
			nodo = {"id": json_revieved['nodes'][i], "symbol": json_revieved['nodes'][i], "size": 12, "bonds": 1,};
			nodesList.push(nodo)
		}
		return nodesList;
	},

	get_links: function(e, nodesList){
		json_revieved = JSON.parse(e.data)

		Object.defineProperty(Object.prototype, "IndexOfDictionary", { 
			value: function(value) {
				for (var key in this){
					if (this[key]["id"] == value)
						return parseInt(key);
				}
				return undefined;
			}
		});

		var linksList = []; 
		// create an array with edges
		for(i = 0; i < Object.keys(json_revieved['edges']).length; i++){
			if(json_revieved['edges'][i].label!=("/")){
		    	edge = {"source": nodesList.IndexOfDictionary(json_revieved['edges'][i].start),
		     			"target": nodesList.IndexOfDictionary(json_revieved['edges'][i].end),
		     			"text": json_revieved['edges'][i].label,
		     		  	"bondType": 1,
		     		  	"id": i
		     		  	};
		    	linksList.push(edge);
			}
		}  
		console.log(linksList)
		return linksList;
	},

	set_camera_topic: function(s){
		console.log(_thisGraph._bondSelected_image)
		_thisGraph._bondSelected_image.attr("xlink:href", function(d) { return ""; })
							    .attr("x", "0px")
							    .attr("y", "0px")
							    .attr("width", "320px")
							    .attr("height", "240px")
							    .attr("xlink:href", "http://10.211.55.17:8080/cam.mjpeg");
	},

	tick: function () {
	  	//Update old and new elements
	    _thisGraph._link.selectAll("line")
	        .attr("x1", function(d) { return d.source.x; })
	        .attr("y1", function(d) { return d.source.y; })
	        .attr("x2", function(d) { return d.target.x; })
	        .attr("y2", function(d) { return d.target.y; });

	    _thisGraph._link.selectAll("text")
	        .attr("x", function(d) { return (d.source.x + d.target.x)/2; })
	        .attr("y", function(d) { return (d.source.y + d.target.y)/2; })

	    _thisGraph._link.selectAll("image")
	        .attr("x", function(d) { return (d.source.x + d.target.x)/2 - 320/2;})
	        .attr("y", function(d) { return (d.source.y + d.target.y)/2 - 240/2;})    

	    _thisGraph._node.attr("transform", function(d) {return "translate(" + d.x + "," + d.y + ")"; });
	},

	draw_graph: function(){

		var color = d3.scale.category20();
		var radius = d3.scale.sqrt().range([0, 6]);

		var atomClicked = function (dataPoint) {
		 	_thisGraph._atomSelected = d3.select(this)
						 		 .select("circle")
						    	 .style("filter", "url(#selectionGlove)")
						    	 .style("fill", "rgb(31, 119, 0)")
						    	 .attr("r", function(d) { return radius(200); });
		};

		var mouseout = function (dataPoint) {
		 	d3.select(this)
				.select("circle")
				.style("filter", "")
				.style("fill", "rgb(31, 119, 180)")
				.attr("r", function(d) { return radius(12); });

		};
		var bondClicked = function (dataPoint) {
		 	Messenger().post({
					  message: 'New Bond Selected',
					  type: 'info',
					  hideAfter: 3,
					  showCloseButton: true
			});
		 	
		 	if (_thisGraph._bondSelected){
		 		_thisGraph._bondSelected.style("filter", "");
				_thisGraph._bondSelected_image.attr("xlink:href", "");
		 	}

		 	_thisGraph._bondSelected =  d3.select(this)
								    .select("line")
									.style("filter", "url(#selectionGlove)")

			_thisGraph._bondSelected_image  = d3.select(this)
									 .select("image");

     		//Bind the same click behavior to all three
			//d3.select(this).each(fadeToFront);
			console.log("\"{'command': ['topic', '" + dataPoint.text + "'] }\"")
	    	socket.send("\"{'command': ['topic', '" + dataPoint.text + "'] }\"")
		};

		var selectionGlove = glow("selectionGlove").rgb("#0000A0").stdDeviation(7);

		var svg = d3.select("#moleculeDisplay").append("svg")
					.attr("width", _thisGraph._width)
					.attr("height", _thisGraph._height)
					.call(selectionGlove);

		var force = d3.layout.force()
						.nodes(nodesList)
						.links(linksList)
						.size([_thisGraph._width, _thisGraph._height])
						.charge(-400)
						.linkStrength(function (d) { return d.bondType * 8;})
						.linkDistance(function(d) { return radius(d.source.size) + radius(d.target.size) + 350; })
						.on("tick", this.tick);


		svg.append("defs").selectAll("marker")
			    .data(["suit", "licensing", "resolved"])
			  .enter().append("marker")
			    .attr("id", function(d) { return d; })
			    .attr("viewBox", "0 -5 10 10")
			    .attr("refX", 25)
			    .attr("refY", 0)
			    .attr("markerWidth", 6)
			    .attr("markerHeight", 6)
			    .attr("orient", "auto")
			  .append("path")
			    .attr("d", "M0,-5L10,0L0,5 L10,0 L0, -5")
			    .style("stroke", "#4679BD")
			    .style("opacity", "1");

			    
		var links = force.links(),
		    nodes = force.nodes();

		_thisGraph._node = svg.selectAll(".node");
	    _thisGraph._link = svg.selectAll(".link");

	  	// Update link data
  		_thisGraph._link = _thisGraph._link.data(links, function (d) {return d.id; });

		  // Create new links
		_thisGraph._link.enter().insert("g", ".node")
		    .attr("class", "link")
		     .style("marker-end",  "url(#suit)") // Modified line 
		    .each(function(d) {
		      	// Add bond line
		      	d3.select(this)
		      	  .append("line")
	              .style("stroke-width", function(d) { return (d.bondType * 3 - 2) * 2 + "px"; });

		      	d3.select(this)
	   	      	  .append("text")
				  .attr("dy", ".35em")
				  .attr("text-anchor", "middle")
				  .attr("fill", "red")
		      	  .text(function(d){
	                    return d.text;
	                });
	            d3.select(this).append("image");

				// Give bond the power to be selected
				d3.select(this)
				  .on("click", bondClicked);
			}); 

		  	// Delete removed links
			_thisGraph._link.exit().remove(); 	
		    // Update node data
		  	_thisGraph._node = _thisGraph._node.data(nodes, function (d) {return d.id; });

		    // Create new nodes
			_thisGraph._node.enter().append("g")
			  .attr("class", "node")
			  .each(function(d) {
				// Add node circle
				d3.select(this)
				  .append("circle")
				  .attr("r", function(d) { return radius(d.size); })
				  .style("fill", function(d) { return color(d.symbol); });

				// Add atom symbol
				d3.select(this)
				  .append("text")
				  .attr("dy", ".35em")
				  .attr("text-anchor", "middle")
				  .text(function(d) { return d.symbol; });

				// Give atom the power to be selected
				d3.select(this)
				  .on("click", atomClicked);

	  			// Give atom the power to be selected
				//d3.select(this)
				//  .on("mouseover", mouseover);		
				d3.select(this)
				  .on("mouseout", mouseout);  

				// Grant atom the power of gravity	
				d3.select(this)
				  .call(force.drag);
			});

		// Delete removed nodes
    	_thisGraph._node.exit().remove();
		force.start();

		//console.log(d3.select(link));
	}
};
