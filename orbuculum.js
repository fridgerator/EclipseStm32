// npm install chart.js
// npm install serialport
// sudo apt-get install libudev-dev
// npm install usb

// npm install -g socket.io







// start with: node usb-chart.js



// npm install express
// npm install socket.io
// npm install @types/socket.io-client --save
// npm install socket.io-client --save
// npm install tail
// npm install sleep

//idVendor           0x0483 STMicroelectronics
//idProduct          0x5740 STM32F407


'use strict';



var http = require('http');
var fs = require('fs');
var path = require('path');

var chartData = [];

var app = require('express')();
var http = require('http').Server(app);
var express = require('express');

const io = require('socket.io')(http);


app.get('/', function(req, res){
  res.sendFile(__dirname + '/index.html');
});
app.use(express.static('.'));


io.on('connection', function(socket){
  console.log('a user connected');
});
http.listen(3000, function(){
  console.log('listening on *:3000');
});


var socket = require('socket.io-client')('http://localhost');

socket.on('connect', function(){});
socket.on('event', function(data){});
socket.on('disconnect', function(){});



    
var fileName = '/home/klemen/temp/out.txt';
var sleep = require('sleep');


// /home/klemen/bin/armgcc/bin/arm-none-eabi-gdb -x /home/klemen/git/EclipseStm32/.mygdbinit
const fork0 = require('child_process').spawn;
var fork1;
var fork2;
const gdb = fork0('/home/klemen/bin/armgcc/bin/arm-none-eabi-gdb',['-x', '/home/klemen/git/EclipseStm32/.mygdbinit'], { silent: true });

gdb.stdout.on('data', function(data) {
    console.log('gdb stdout: ' + data);
    if(!fork1 && data.indexOf('HAL_Init') !== -1)
    {
      fork1 = require('child_process').spawn;
      // /home/klemen/git/orbuculum/ofiles/orbuculum -c 0,capValue,\"%d\" -v 2
      console.log('Starting ORBUCULUM');
      const orbuculum = fork1('/home/klemen/git/orbuculum/ofiles/orbuculum',['v', '2', 'c', '0,capValue,\\"%d\\"'], { silent: true });
      
      orbuculum.stdout.on('data', function(data1) {
	console.log('orbuculum stdout: ' + data1);
	if(!fork2){
	  fork2 = require('child_process').spawn;
	  console.log('Starting ORBCAT');
	  // /home/klemen/git/orbuculum/ofiles/orbcat -c '0,\"%d\"\n' > fileName
	  const orbcat = fork2('/home/klemen/git/orbuculum/ofiles/orbcat', ['-c', '0,%d\n', '>', fileName], { silent: true });  
	  
	  orbcat.stdout.on('data', function(data2){
	    console.log('orbcat stdout: ' + data2);
	    
	  });
	  orbcat.stderr.on('data', function(data3){
	    console.log('orbcat err: ' + data3);
	  });
	  orbcat.on('close', function(code1){
	    console.log('orbcat child process exited with code ' + code1);
	  });
	}
	
      });
      orbuculum.stderr.on('data', function(data4){
	console.log('orbuculum err:' + data4);
      });
      orbuculum.on('close', function(code2){
	console.log('orbuculum child process exited with code ' + code2);
      });
    }
});
gdb.stderr.on('data', function(data5){
  console.log('gdb err: ' + data5);
});
gdb.on('close', function(code3){
  console.log('gdb child process exited with code ' + code3);
});



// shuf -i X-Y -n 20 > /home/klemen/temp/out.txt	

var Tail = require('tail').Tail;
var tail = new Tail(fileName);
tail.on("line", function(data) {
  //console.log(data);
  io.emit('message',data);
});
 
tail.on("error", function(error) {
  console.log('ERROR: ', error);
});


