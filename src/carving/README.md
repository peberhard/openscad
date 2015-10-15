
# OpenSCAD carving plugin

See https://github.com/peberhard/openscad-carving-plugin-doc-examples for documentation and examples.

## Build

Follow [OpenSCAD build instructions](https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Building_OpenSCAD_from_Sources) and add CONFIG+="carving" to the qmake command. Example:

    $ qmake CONFIG+="experimental carving"
    $ make


## carving plugin overview

- new keywords in language are prefixed by "carving_".
- carving_modules handles the instantiation of new nodes related to the keywords.
- once the node tree has been generated from ASP, a specifc traversor updates the carving context of carving nodes.
- Rendering follows OpenSCAD design: Geometry nodes inherit the LeafNode and constructive nodes inherit the CsgNode.
- export in G-Code is made in 2 steps: A G-Code tree is generated from the node tree, then this G-Code tree is traversed to generate the G-Code file.


## Known bugs

* didn't got syntax colorisation working yet


## Backlog

* Make carving 'plugin' activable using features?
* Render accurately drill bit with front angle http://wiki.linuxcnc.org/cgi-bin/wiki.pl?ToolTable
* Fold parts. Usage: Make a thin line cut in aluminum part to help folding with a drill bit of 45° front angle. Take into account bend deduction.
  https://en.wikipedia.org/wiki/Bending_%28metalworking%29#Bend_deduction
* Generate G-Code following a 2d primitives 
* Text support
* Generate G-Code following 3d volume (by slicing+offset?)
* Optimize G-Code once G-Code tree is generated (minimize moves and optimize change of tool)
* Estimate milling duration of a project (from G-Code)

