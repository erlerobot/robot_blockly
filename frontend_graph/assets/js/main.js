(function () {
	var width = 1160,
	    height = 600;

	var color = d3.scale.category20();
	var radius = d3.scale.sqrt()
	    .range([0, 6]);

	var atomSelected;
	var bondSelected;
	var atomClicked = function (dataPoint) {
	 	if (dataPoint.symbol === "H")
	 		return;

	 	atomSelected = d3.select(this)
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
	 	
	 	if (bondSelected)
	 		bondSelected.style("filter", "");

	 	bondSelected = d3.select(this)
						 .select("line")
				    	 .style("filter", "url(#selectionGlove)");
    	console.log(bondClicked)
	};

	var selectionGlove = glow("selectionGlove").rgb("#0000A0").stdDeviation(7);

	var svg = d3.select("#moleculeDisplay").append("svg")
				.attr("width", width)
				.attr("height", height)
				.call(selectionGlove);

	linksList = [
	            {"source": 0, "target": 1,  "bondType": 1, "id": 21, "text": "topic2"},
	            {"source": 0, "target": 2,  "bondType": 1, "id": 22, "text": "topic1"}
            	];
    nodesList = [
            	{"symbol": "C", "size": 12, "bonds": 1, "id":1},
            	{"symbol": "C", "size": 12, "bonds": 1, "id":2},
            	{"symbol": "C", "size": 12, "bonds": 2, "id":3}
            	];

	var force = d3.layout.force()
					.nodes(nodesList)
					.links(linksList)
					.size([width, height])
					.charge(-400)
					.linkStrength(function (d) { return d.bondType * 1;})
					.linkDistance(function(d) { return radius(d.source.size) + radius(d.target.size) + 100; })
					.on("tick", tick);

	var links = force.links(),
	    nodes = force.nodes(),
	    link = svg.selectAll(".link"),
	  	node = svg.selectAll(".node");

	  	// Update link data
  	link = link.data(links, function (d) {return d.id; });

		  // Create new links
	link.enter().insert("g", ".node")
	    .attr("class", "link")
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
			// If double add second line
			d3.select(this)
				.filter(function(d) { return d.bondType >= 2; }).append("line")
				.style("stroke-width", function(d) { return (d.bondType * 2 - 2) * 2 + "px"; })
				.attr("class", "double");

			d3.select(this)
				.filter(function(d) { return d.bondType === 3; }).append("line")
				.attr("class", "triple");

			console.log("lol")
			// Give bond the power to be selected
			d3.select(this)
			  .on("click", bondClicked);
		}); 

	  	// Delete removed links
		link.exit().remove(); 	
	    // Update node data
	  	node = node.data(nodes, function (d) {return d.id; });

	    // Create new nodes
		node.enter().append("g")
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
    node.exit().remove();
	force.start();

	console.log("function main.js")
	function tick() {
	  	//Update old and new elements
	    link.selectAll("line")
	        .attr("x1", function(d) { return d.source.x; })
	        .attr("y1", function(d) { return d.source.y; })
	        .attr("x2", function(d) { return d.target.x; })
	        .attr("y2", function(d) { return d.target.y; });

	    link.selectAll("text")
	        .attr("x", function(d) { return (d.source.x + d.target.x)/2; })
	        .attr("y", function(d) { return (d.source.y + d.target.y)/2; })

	    node.attr("transform", function(d) {return "translate(" + d.x + "," + d.y + ")"; });
	  }
})();