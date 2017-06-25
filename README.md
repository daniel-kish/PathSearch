# PathSearch

## About
Here a collection of [path/motion planning](https://en.wikipedia.org/wiki/Motion_planning) algortihms are implemeneted. This is a course project where each student was supposed to work on a specific motion planning method. The statement of the problem ran as follows

> Given a polygon with polygon "holes" find a shortest path between any two given points.
> For instance:
![example](/example.png)

## Structure
At first a naive yet stable version of a Delaunay triangulation was implemented. That was used both in an obstacle generator, which was the main initial goal of the work. Then three path planning algorithms were developed:
1. Delaunay triangulation motion planning
2. Voronoi diagramm motion planning
3. A variant of Lee algorithm
The first one plans a path for a point, i.e. neglecting possible issues with a size of the moving agent. Any area with obstacles can be preprocessed into a triangulation like shown below:

![obstacles](/obstacles.png) ![triangulation](/triangulation_path.png)

Then with any decent implementation of a broadth-first or A* search algorithm one can obtain shortest path from any point toany other. Path will follow the obstacles sides, which makes sense, since agent's size is set to zero. 
Although when an object has a non-zero size, we can use the very same technique to provide a path for it in the same circumstances. And for that purpose we use Voronoi diagram, like this:

![voronoi](/path_voronoi.png)
