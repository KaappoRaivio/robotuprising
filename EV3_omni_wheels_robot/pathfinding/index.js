/*
We are using Pathfinding library to find the optimal path between obstacles. The library can be found in https://github.com/qiao/PathFinding.js/tree/master

We use two methods: 
1) Building the UI locally and exporting the path from a drawing
2) Taking photos of the track and converting into path matrix

Once otpimal path is extracted, it is converted to the coornidates which the omniwheel bot can understand.
*/

var mqtt = require('mqtt')
//const { path } = require('./findPath')
var client  = mqtt.connect('mqtt://iot.eclipse.org:1883')
//console.log(path)

/*const path = [[37,93],[37,92],[37,91],[37,90],[37,89],[37,88],[38,87],[39,86],[40,85],[41,85],[42,85],[43,85],[44,85],[45,85],[46,85],[47,84],[48,84],[49,84],[49,83],[49,82],[49,81],[49,80],[49,79],[49,78],[49,77],[49,76],[49,75],[49,74],[49,73],[49,72],[49,71],[49,70],[48,69],[47,68],[46,67],[45,66],[44,65],[43,64],[42,63],[41,62],[40,61],[39,61],[38,60],[37,59],[36,59],[36,58],[35,57],[34,56],[33,55],[32,54],[31,53],[30,52],[30,51],[30,50],[30,49],[30,48],[31,47],[32,46],[33,45],[34,44],[35,44],[36,44],[37,43],[38,43],[39,43],[40,43],[41,42],[42,42],[43,42],[44,42],[45,42],[46,42],[47,42],[48,41],[49,40],[50,40],[51,40],[52,39],[53,39],[54,39],[55,38],[56,37],[57,36],[58,36],[59,35],[60,34],[60,33],[60,32],[60,31],[60,30],[60,29],[60,28],[60,27],[60,26],[60,25],[59,24],[58,23],[57,22],[56,22],[55,22],[54,21],[53,20],[52,19],[51,18],[50,17],[49,16],[48,15],[47,14],[46,13],[45,12],[44,11],[43,10],[42,9],[41,8],[40,7],[39,6],[38,5],[37,4],[36,3],[36,2],[36,1]]
*/

const formatPath = (path) => {
    // Format path from absolute coordinates to delta
    const newPath = []
    for(let i = 0; i < path.length - 1; i++) {
        const x = path[i].map((item, index) => {
            return path[i+1][index] - item;
            
        })
        newPath.push(x)
    }
    return newPath
}

const compressPath = (data) => {
    // Compress path (sequential similar moves are compressed into one)
    let previous = data[0]
    let sum = data[0]
    const out = []
    for (let i=1; i < data.length; i++) {
        if(JSON.stringify(data[i]) === JSON.stringify(previous)) {
            if (!sum) {
                sum = data[i]
            }
            else {
                sum[0] += data[i][0]
                sum[1] += data[i][1]
                previous = data[i]
            }
        }
        else {
            out.push(sum)
            sum = data[i]
            previous = data[i]
        }
    }
    out.push(sum)
    return out
}

let time

const roboMap = (data) => {
    // Map compressed path into robo coordinates
    const mult = 20 // Multiplier to map coordinates to measurement units
    const coords = data.map(item => [item[0] * mult, item[1] * mult])
    const commands = coords.map(item => {
        const x = item[0].toFixed(1)
        const y = item[1].toFixed(1)
        // Calculate optimal running time
        time = (Math.sqrt(Math.pow(x,2)+Math.pow(y,2))/100).toFixed(1)
        return `x ${x} y ${(-1)*y} rot 0.0 dt ${time}`
    })
    return commands
}


const formatted = formatPath(path)
console.log('formatted', formatted)
console.log('formatted path length', formatted.length)
const compressed = compressPath(formatted)
console.log('compressed', compressed)
console.log('compressed path length', compressed.length)
const robomapped = roboMap(compressed)
console.log(robomapped)

function sleep(milliseconds) {
    // Sleeping period between different commands
    console.log('sleeping')
    var start = new Date().getTime();
    for (var i = 0; i < 1e7; i++) {
      if ((new Date().getTime() - start) > milliseconds){
        break;
      }
    }
  }
  

client.on('connect', function () {
    // Connect to mqtt server to send commands to the robot
    console.log('connected')

    robomapped.forEach(coordinate => {
        const message = coordinate
        console.log(message)
        const waitTime = parseFloat(message.split(' ')[7])*1000
        console.log("waitTime:(ms)", waitTime)
        client.publish('local', message)
        sleep(waitTime);
    })
    client.end()
})
