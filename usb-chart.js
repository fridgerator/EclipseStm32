// npm install chart.js
// npm install serialport
// npm install socket.io
// npm install -g socket.io
// npm install @types/socket.io-client --save
// npm install express




//idVendor           0x0483 STMicroelectronics
//idProduct          0x5740 STM32F407


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
http.listen(8080, function(){
  console.log('listening on *:3000');
});


var socket = require('socket.io-client')('http://localhost');

socket.on('connect', function(){});
socket.on('event', function(data){});
socket.on('disconnect', function(){});

const SerialPort = require('serialport');
const parsers = SerialPort.parsers;

// Use a `\r\n` as a line terminator
const parser = new parsers.Readline({
  delimiter: '\n'
});

const port = new SerialPort('/dev/ttyACM0', {
  baudRate: 115200
});

port.pipe(parser);
port.on('open', () => console.log('Port open'));


parser.on('data', function(data) {
    //console.log(data);
    io.emit('message',data);
});

