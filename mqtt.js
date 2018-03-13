

var mqtt = require('mqtt')
var client  = mqtt.connect('mqtt://kay.zyxel.me')
 
client.on('connect', function () {
  client.subscribe('simplecare/#')
  //client.publish('presence', 'Hello mqtt')
})
 
client.on('message', function (topic, message) {
  // message is Buffer
  console.log(message.toString())
  //client.end()
})

