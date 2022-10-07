var express = require('express')
const bodyParser = require("body-parser")
  
const app = express();
app.use(bodyParser.urlencoded({
    extended:true
}));

// Listens for obstacles info
app.get('/set-obstacles/', function(req, res) {
  console.log('GET /')

  // Get obstacles and starting position
  var obstacle_str = req.query.str
  var init_str = req.query.init

  // Spawns python process for computing path with the receieved data
  const spawn = require("child_process").spawn;
  console.log("sending cmd => " + 'python movement.py ' + obstacle_str + ' ' + init_str);
  const pythonProcess = spawn('python',["movement.py", obstacle_str, init_str]);
  pythonProcess.stdout.on("data", data => {
    console.log(`stdout: ${data}`);
  });
  res.send(obstacle_str)
})

app.listen(80, function(){
  console.log("server is running on port 80");
})