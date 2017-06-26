# PathSearch

## About
Here a collection of [path/motion planning](https://en.wikipedia.org/wiki/Motion_planning) algortihms are implemeneted. This is a course project where each student was supposed to work on a specific motion planning method. The statement of the problem ran as follows

> Given a polygon with polygon "holes" find a shortest path between any two given points.
> For instance:
![example](/example.png)

## Structure
At first a naive yet stable version of a Delaunay triangulation was implemented. That was used both in an obstacle generator, which was the main initial goal of the work. Voronoi diagramm motion planning algorithm was developed. It approximates the full-blown generalised Voronoi diagramm of the obstacles with VD of only their vertices which is most of the time enough for paths that are somewhere near the precise path.
![obstacles](/obstacles.png) ![triangulation](/triangulation_path.png)

Then with any decent implementation of a broadth-first or A\* search algorithm one can obtain shortest path from any point to any other. Path will follow the obstacles sides, which makes sense, since agent's size is set to zero. 
Although when an object has a non-zero size, we can use the very same technique to provide a path for it in the same circumstances. And for that purpose we use Voronoi diagram, like this:

![voronoi](/path_voronoi.png)

Here's another example of an area with obstacles, much bigger than the previous and it also looks more labyrinthine.
![big_example](/big_example.png)

The main course of of work in this project was dedicated to proper obstacle generation. The thing is that most generators use a comparatively simple logic to build obstacles. They merely generate star shaped polygons and scatter them around a given area. Furthermore the obstacles better not intersect since there is a lot of motion planning algorithms out there and the admissibility of intersecting obstacles can be argued, some of algorithms are not tolerant to something like this:
![wrong](/wrong.png)

The obstacle generator developed in this project uses a Delaunay triangulation of some (preferably dense) set of points that can be scattered or organised in a nice grid. Based on the triangulation algorithm "grows" a set of areas made of triangles. Those areas never intersect each other and can be set up to be very long and unpredictable or very compact and close to a convex polygon. All examples above were generated with the developed program. 
Here we can see what will happen of we use homogeneous grid instead of the scattered set.

![non_compact](non_compact_grid.png)

And here we make obstacles a little more compact, broadth-first rather than depth first, so to say.
![compact](/compact_grid.png)
