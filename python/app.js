const express = require('express')();
const http = require('http').Server(express);
app.get('/', (req, res) => {
	res.sendFile(path.join(__dirname + '/index.htm'))
})
http.listen(3000, function () {
	console.log('Listening on port 3000!')
})