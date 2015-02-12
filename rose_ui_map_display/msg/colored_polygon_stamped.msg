std_msgs/Header header
geometry_msgs/Polygon polygon
#A Polygon is an array of points, between which lines are drawn. 
#colors is also a list, which indicates per point what color the line from that point to the next point should be.
#This means the colors-array has one less item thant the points-arry of polygon, 
#   because the last point will not have a next point between which to draw a colored line. 
std_msgs/ColorRGBA[] colors