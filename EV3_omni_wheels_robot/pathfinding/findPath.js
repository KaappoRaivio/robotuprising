/*
Testfile for exporting path from matrix obtained from photo
*/
const PF = require('pathfinding');
const { matrix } = require('./asdasd')

const grid = new PF.Grid(matrix)

const finder = new PF.AStarFinder({
    allowDiagonal: true,  // Allow also diagonal movement
    dontCrossCorners: true // Prevent crossing corners
});

const path = finder.findPath(125, 201, 135, 0, grid)