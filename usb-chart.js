// npm install chart.js
// npm install serialport
// sudo apt-get install libudev-dev
// npm install usb
// npm install socket.io
// npm install -g socket.io
// npm install @types/socket.io-client --save
// npm install express

// start with: node usb-chart.js


//idVendor           0x0483 STMicroelectronics
//idProduct          0x5740 STM32F407



// /usr/local/n/versions/node/9.2.1/bin/npm rebuild


// debugging:
// nodejs --inspect --debug-brk ./usb-chart.js
// then open in chrome browser: chrome://inspect
// 	and step through code


'use strict';

// Use a Readline parser

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





// serial port version START



const SerialPort = require('serialport');
var port;

var comName = '';
SerialPort.list((err, ports) => {
  console.log("error: " + err);
  var foundPort = false;
  ports.forEach((port) => {
    console.log(port.vendorId + " " + port.productId + " " + port.comName + " condition: " + (port.vendorId == "0483" && port.productId== "5740") + " commName: " + port.comName);
    var comName=port.comName
    if (port.vendorId == "0483" && port.productId== "5740") {
      console.log(comName);
      foundPort = true;
      
      const parsers = SerialPort.parsers;

      // Use a `\r\n` as a line terminator
      const parser = new parsers.Readline({
	delimiter: '\n'
      });

      const port = new SerialPort(comName, {
	baudRate: 115200
      });

      port.pipe(parser);
      port.on('open', () => console.log('Port open'));

      parser.on('data', function(data) {
	//console.log(data);
	if(data.startsWith("ButtonControl"))
	  console.log(data);
	else
	  io.emit('message',data);
      });      
      
      
    }
  });
});



// serial port version END




/*
const usb = require("usb");
usb.setDebugLevel(4);
var dev = usb.findByIds(0x0483, 0x5740);

console.log("Descriptor_______________");
console.log(dev.configDescriptor);


console.log("OPEN____________");
dev.open();


var iface = dev.interfaces[0];
var driverAttached = false;

if (iface.isKernelDriverActive()) {
   driverAttached = true;
   iface.detachKernelDriver();
}


console.log("Interfaces_______________");
console.log("Kernel driver active? " + iface.isKernelDriverActive());


iface.claim();
var inEndpoint = iface.endpoints[0];

console.log("InEndpoint descriptor______");
console.log(inEndpoint.descriptor);

inEndpoint.startPoll(1, 1);
inEndpoint.on('data',function(d)
{
  console.log(d);
}
);
inEndpoint.on('error',function(err)
{
  console.log("Pol error" + err);
}
);
inEndpoint.on('end',function()
{
  console.log("Pol END");
}
);

*/