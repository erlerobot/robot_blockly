function glow(url) {
  var stdDeviation = 2,
      rgb = "#1F75C4",
      colorMatrix = "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0";
 
  if (!arguments.length) {
    url = "glow";
  }
 
  function my() {
 
    var defs = this.append("defs");
 
    var filter = defs.append("filter")
        .attr("id", url)
        .attr("x", "-20%")
        .attr("y", "-20%")
        .attr("width", "140%")
        .attr("height", "140%")
      .call(function() {
        this.append("feColorMatrix")
            .attr("type", "matrix")
            .attr("values", colorMatrix);
        this.append("feGaussianBlur")
             // .attr("in", "SourceGraphics")
            .attr("stdDeviation", stdDeviation)
            .attr("result", "coloredBlur");
      });
 
    filter.append("feMerge")
      .call(function() {
        this.append("feMergeNode")
            .attr("in", "coloredBlur");
        this.append("feMergeNode")
            .attr("in", "SourceGraphic");
      });
  }
 
  my.rgb = function(value) {
    if (!arguments.length) return color;
    rgb = value;
    color = d3.rgb(value);
    var matrix = "0 0 0 red 0 0 0 0 0 green 0 0 0 0 blue 0 0 0 1 0";
    colorMatrix = matrix
      .replace("red", color.r/256)
      .replace("green", color.g/256)
      .replace("blue", color.b/256);
    return my;
  };
 
  my.stdDeviation = function(value) {
    if (!arguments.length) return stdDeviation;
    stdDeviation = value;
    return my;
  };
 
  return my;
}