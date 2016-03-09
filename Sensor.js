// ssh root@192.168.7.2
"use strict"
var Skeleton = require("../skeleton.js");

Sensor.prototype = new Skeleton("SENSOR");
Sensor.prototype.constructor = Sensor;

function Sensor(model_ref, feedback, debug) {
	this.model = model_ref;
	this.feedback = feedback;
	this.debug = false;

	//interval
	this.intervals = {
		queues: {
			compass: undefined,
			gyro: undefined,
			accelero: undefined,
			temp: undefined,
			signal: undefined
		},
		periods: {
			compass: 500,
			gyro: 500,
			accelero: 500,
			temp: 2000,
			signal: 3000
		}
	}
	this.I2C = {
		compass: undefined,
		gryo: undefined,
		accelerometer: undefined
	};

	global.XAXIS = 0;
	global.YAXIS = 1;
	global.ZAXIS = 2;

	var SerialPort = SERIALPORT.SerialPort; // make a local instant
	this.gpsPort = new SerialPort("/dev/ttyO1", { // <--Then you open the port using new() like so
			baudRate: 9600,
			parser: SERIALPORT.parsers.readline("\r\n") // look for return and newline at the end of each data packet
	});
	this.AuxillaryPort = new SerialPort("/dev/ttyO2", { // <--Then you open the port us$
		baudRate: 9600,
		parser: SERIALPORT.parsers.readline("\r\n") // look for return and newl$
	});
	this.buffer = new Buffer(100);
	//initiate
	this.initGYRO();
	this.initACCELEROMETER();
	this.initCOMPASS();
	this.initGPS();
	// [power,voltage,potentiometer]
	this.initAUXPORT();    
	this.initTemp();
	this.initSignalTracker();
};

Sensor.prototype.handle = function(data) { // take command from user interface
	console.log(this.module + " Recieved ", data);
	//start command 
	switch(data) {
		case "START-ALL":
			this.initGYRO();
			this.initACCELEROMETER();
			this.initCOMPASS();
			this.initGPS();
			return "GATHERING SENSOR DATA";
		case "START-GRYO":
			this.initGYRO();
			return "STARTING GRYO";
		case "START-ACCELEROMETER":
			this.initACCELEROMETER();
			return "STARTING ACCELEROMETER";
		case "START-COMPASS":
			this.initCOMPASS();
			return "STARTING COMPASS";
		case "START-GPS":
			this.initGPS();
			return "STARTING GPS";
		case "STOP-ALL":
			clearInterval(this.intervals.queues.compass);
			clearInterval(this.intervals.queues.gyro);
			clearInterval(this.intervals.queues.accelero);
			clearInterval(this.intervals.queues.temp);
			clearInterval(this.intervals.queues.signal);
			return "STOPPING ALL SENSORS";
		case "STOP-GRYO":
			clearInterval(this.intervals.queues.gyro);
			return "STOPPING GRYO";
		case "STOP-ACCELEROMETER":
			clearInterval(this.intervals.queues.accelero);
			return "STOPPING ACCELEROMETER";
		case "STOP-COMPASS":
			clearInterval(this.intervals.queues.compass);
			return "STOPPING COMPASS";
		case "STOP-GPS":
			return "STOPPING GPS*";
		case "STOP-TEMP":
			clearInterval(this.intervals.queues.temp);
			return "STOPPING GPS*";
		case "STOP-SIGNAL":
			clearInterval(this.intervals.queues.signal);
			return "STOPPING GPS";
		case "MAST-UP":
			this.model.acuator.sent_position = "U";
			this.acuator();
			return "MAST GOING UP";
		case "MAST-DOWN":
			this.model.acuator.sent_position = "D";
			this.acuator();
			return "MAST GOING DOWN";
		default:
			return "INVALID SENSORS COMMAND GIVEN: " + data;
	}
};

Sensor.prototype.initCOMPASS = function() { // degrees refer to North
	var parent = this;
	clearInterval(this.intervals.queues.compass);
	
	if(_.isUndefined(this.I2C.compass)) {
		try { 
			var address_compass = 0x1e; //address of compass
			this.I2C.compass = new I2C(address_compass, {
				device: '/dev/i2c-2'
			});
		} catch(err){
			console.log("error", err);
			this.feedback(this.module, "COMPASS FAILED TO INITIALIZE!");
			return;
		}	
	}
	this.I2C.compass.writeBytes(0x00, [0x70], function(err) {});
	this.I2C.compass.writeBytes(0x01, [0xA0], function(err) {});
	//countinuous read mode
	this.I2C.compass.writeBytes(0x02, [0x00], function(err) {}); 

	this.intervals.queues.compass = setInterval(function() {
		parent.I2C.compass.readBytes(0x03, 6, function(err, res) {
			var X = 0;
			var Y = 0;
			var z = 0;
			if (!err) {
				// convert binary to signed decimal 
				X = new Int16Array([res[0] << 8 | res[1]])[0]; //put binary into an array and called back the first numer
				z = new Int16Array([res[2] << 8 | res[3]])[0];
				Y = new Int16Array([res[4] << 8 | res[5]])[0];
			} else {
				console.log("Compass Error ::: " + JSON.stringify(err));
				parent.feedback(parent.module, "COMPASS ERROR, STOPPING COMPASS!");
				clearInterval(parent.intervals.queues.compass);
			}
			//routine to give a fast solution for angle, from X/Y co-ordinates - result in degrees 
			var AX,AY,ival,oval,aival,quad; 
			AX=Math.abs(X);
			AY=Math.abs(Y);
			//Now the approximation used works for tan from -1 to 1, so we have to keep the 
			//values inside this range and adjust the input/output. 
			//Four 'quadrants' are decoded -1 to 1, (315 to 45 degrees), then 45 to 135, 
			//135 to 225, 225 to 315
			//Right hand half of the circle
			try {
				if (X >= 0) { 
					if (AY > X) { 
						if (Y < 0) { 
							quad = 4; 
							ival = -X / Y; 
						} else { 
							quad = 2; 
							ival = X / -Y; 
						} 
					} else { 
						if (AY > X) { 
							quad = 4; 
							ival = -Y / X; 
						} else { 
							quad = 1; 
							ival = Y / X; 
						} 
					}
				} else { 
					if (Y > AX) { 
						quad = 2; 
						ival = X / -Y; 
					} else { 
						if (AY > AX) {          
							quad = 4; 
							ival = -X / Y; 
						} else { 
							quad = 3; 
							ival = -Y / -X; 
						}
					}
				}	
			} catch (e) {
				parent.feedback("Compass Division Error, NOT stopping event, error is = " +e);
				parent.model.compass.heading = 0;
			}
			
			//A lot of lines of code, but small and quick really..... 
			//Now the solution 
			//Now approximation for atan from -1 to +1, giving an answer in degrees. 
			aival = Math.abs(ival); 
			oval = 45 * ival - ival * (aival - 1) * (14.02 + 3.79 * aival); 
			//Now solve back to the final result 
			if (quad != 1) 
			{ 
				if (quad == 2) { 
					oval = oval + 90; 
				} else { 
					if (quad == 3) {
						oval = oval + 180; 
					} else { 
						oval = oval + 270; 
					}
				} 
			}
			// Adding 360
			if (oval<0) { oval+=360; } 
			//Flip around
			oval = Math.abs(oval - 360);
			parent.model.compass.heading = oval;
			if(parent.debug) {
				console.log('Heading: ' + parent.model.compass.heading + ' degrees');	
			}
		});
	}, this.intervals.periods.compass);
};

Sensor.prototype.initGYRO = function() {
	clearInterval(this.intervals.queues.gyro);
	if(_.isUndefined(this.I2C.gyro)) {
		try { 
			var address_gyroscope = 0x68; //address of gyroscope
			this.I2C.gryo = new I2C(address_gyroscope, {
				device: '/dev/i2c-2'
			});
		} catch(err){
			console.log("error", err);
			this.feedback(this.module, "GYRO FAILED TO INITIALIZE!");
			return;
		}
	}
	var x, y, z;
	var parent = this;

	this.I2C.gryo.writeBytes(0x16, [1 << 3 | 1 << 4 | 1 << 0], function(err) {}); // set rate 2000
	this.I2C.gryo.writeBytes(0x15, [0x09], function(err) {}); // set sample rate to 100hz

	this.intervals.queues.gyro = setInterval(function() {
		parent.I2C.gryo.readBytes(0x1D, 6, function(err, res) {
			if (!err) {
				// convert binary to signed decimal 
				x = new Int16Array([res[0] << 8 | res[1]])[0]; //put binary into an array and called back the first number
				z = new Int16Array([res[2] << 8 | res[3]])[0];
				y = new Int16Array([res[4] << 8 | res[5]])[0];
			} else {
				parent.feedback(parent.module, "GYRO ERROR, STOPPING GYRO!");
				clearInterval(parent.intervals.queues.gyro);
				console.log("Gyro Error ::: " + JSON.stringify(err));
			}
			parent.model.gyro.x = parent.model.gyro.x + ((x) / 14.375) * .1; //to get degrees 
			parent.model.gyro.y = parent.model.gyro.y + ((y) / 14.375) * .1; //
			parent.model.gyro.z = parent.model.gyro.z + ((z) / 14.375) * .1; //
			if (parent.model.gyro.x > 360) {
				parent.model.gyro.x = parent.model.gyro.x % 360;
			}
			if (parent.model.gyro.x < -360) {
				parent.model.gyro.x = parent.model.gyro.x % 360;
			}
			if (parent.model.gyro.y > 360) {
				parent.model.gyro.y = parent.model.gyro.y % 360;
			}
			if (parent.model.gyro.y < -360) {
				parent.model.gyro.y = parent.model.gyro.y % 360;
			}
			if (parent.model.gyro.z > 360) {
				parent.model.gyro.z = parent.model.gyro.z % 360;
			}
			if (parent.model.gyro.z < -360) {
				parent.model.gyro.z = parent.model.gyro.z % 360;
			}
			if (parent.debug){ 
				console.log("pitch: " + parent.model.gyro.x + " roll: " + parent.model.gyro.y + " yaw: " + parent.model.gyro.z + " degrees");
			}
		});
	}, this.intervals.periods.gyro);
};

Sensor.prototype.initACCELEROMETER = function() {
    var ADXL345 = require('./ADXL345.js');
    var parent = this;

    global.XAXIS = 0;
    global.YAXIS = 1;
    global.ZAXIS = 2;

    var globalvar = {
        SAMPLECOUNT: 400,
        accelScaleFactor: [0.0, 0.0, 0.0],
        runTimeAccelBias: [0, 0, 0],
        accelOneG: 0.0,
        meterPerSecSec: [0.0, 0.0, 0.0],
        accelSample: [0, 0, 0],
        accelSampleCount: 0
    };
    var accel = new ADXL345(function(err) {
        accel.accelScaleFactor[XAXIS] = 0.0371299982;
        accel.accelScaleFactor[YAXIS] = -0.0374319982;
        accel.accelScaleFactor[ZAXIS] = -0.0385979986;
        if (!err) {
            computeAccelBias();
        } else {
            console.log(err);
        }
    });

    function computeAccelBias() {
        accel.computeAccelBias(function() {
            measureAccel();
        });
    }

    function measureAccel() {
        parent.intervals.queues.accelero = setInterval(function() {
            accel.measureAccel(function(err) {
                if (!err) {

                    //parent.model.accelero.x = (accel.meterPerSecSec[global.XAXIS]) * (-8.85);
                    //parent.model.accelero.y = (accel.meterPerSecSec[global.YAXIS]) * (8.17);

                    var x = (accel.meterPerSecSec[global.XAXIS]) ;
                    var y = (accel.meterPerSecSec[global.YAXIS]) ;
                    parent.model.accelero.z = accel.meterPerSecSec[global.ZAXIS];

                    parent.model.accelero.y = -0.0583*x*x*x - 0.0471*x*x - 1.9784*x + 0.2597 
                    parent.model.accelero.x =  0.0475*x*x*x + 0.1038*x*x + 3.4858*x + 0.1205 
                } else {
                    console.log(err);
                }
            });
        }, parent.intervals.periods.accelero);
    }
};

Sensor.prototype.initGPS = function() {
	var parent = this;

	if(_.isUndefined(this.gpsPort)) {
		var SerialPort = SERIALPORT.SerialPort; // make a local instant
		this.gpsPort = new SerialPort("/dev/ttyO1", { // <--Then you open the port using new() like so
			baudRate: 9600,
			parser: SERIALPORT.parsers.readline("\r\n") // look for return and newline at the end of each data packet
		});
	}
	this.gpsPort.on('open', function() {
		console.log('[GPS] Port Open. Data Rate: ' + parent.gpsPort.options.baudRate);
		console.log("[GPS] begin initialization"); //begin initialization
		parent.gpsPort.write("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
		parent.gpsPort.write("$PMTK220,200*2C\r\n"); //5hz update
		parent.gpsPort.write("$PMTK300,200,0,0,0,0*2F\r\n"); //    //5hz
		console.log("[GPS] initialization complete!"); //print out to terminal
		parent.feedback(parent.module,"[GPS] initialization complete!"); //print out to terminal
	});
	this.gpsPort.on('close', function() {
		console.log('port closed.');
	});
	this.gpsPort.on('error', function() {
		console.log('Serial port error: ' + error);
	});
	this.gpsPort.on('data', function(data) {
		var piece = data.split(",", 7);
		
		var lat = piece[3];
		var lat_dir = piece[4];
		var lng = piece[5];
		var lng_dir = piece[6];

		parent.model.GPS.longitude = lng;
		parent.model.GPS.latitude = lat;
		parent.model.GPS.longitude_dir = lng_dir;
		parent.model.GPS.latitude_dir = lat_dir;
		
		if(parent.debug) {
			console.log("lat: " + parent.model.GPS.latitude + " long: " + parent.model.GPS.longitude);
		}
	});
};
Sensor.prototype.initAUXPORT = function() {
	var parent = this;

	if(_.isUndefined(this.gpsPort)) {
		var SerialPort = SERIALPORT.SerialPort; // make a local instant
		this.AuxillaryPort = new SerialPort("/dev/ttyO2", { // <--Then you open the port us$
			baudRate: 9600,
			parser: SERIALPORT.parsers.readline("\r\n") // look for return and newl$
		});
	}

	this.AuxillaryPort.open(function(error) {
		if (error) {
			console.log("AUXILLARY ARDUINO PORT FAILED TO OPENED!");
			parent.feedback(parent.module, "AUXILLARY ARDUINO PORT FAILED TO OPENED!");
		} else {
			console.log("AUXILLARY ARDUINO PORT HAS BEEN OPENED");
			parent.feedback(parent.module, "AUXILLARY ARDUINO PORT HAS BEEN OPENED!");
			parent.AuxillaryPort.on('data', function(data) {
				var voltage_string = [""];      //initiate a string
				var current_string = [""];
				var potentiometer_string = [""];                  
				var end_bit = '#'; 

				parent.buffer = data;
				for (var i = 0; i < 20; i++) {
					//current evaluation 
					if (parent.buffer[i] == 'C' ) {
						while (parent.buffer[++i] != end_bit) {
							current_string += parent.buffer[i];   // populate string
							parent.model.power.current = parseFloat(current_string); // change string into float
						}
					}
					//voltage evaluation
					if (parent.buffer[i] == 'V') {
						while (parent.buffer[++i] != end_bit) {
							voltage_string += parent.buffer[i];  // populate string
							parent.model.power.voltage = parseFloat(voltage_string); // change string into float 
						}
					}
					//acuator evaluation 
					if (parent.buffer[i] == 'P') {
						while (parent.buffer[++i] <= end_bit) {
							potentiometer_string += parent.buffer[i];  // populate string
							parent.model.acuator.potentiometer = parseFloat(potentiometer_string); // change string into float 
						}
					}
				}
				if(parent.debug) {
					console.log("voltage: " + parent.model.power.voltage);
					console.log("current: " + parent.model.power.current);
					console.log("potentiometer: " + parent.model.acuator.potentiometer);
				}
			});
		}
   });                       
};

Sensor.prototype.acuator = function() {
	var parent = this;
	//write command to arduino
	this.AuxillaryPort.write(this.model.acuator.sent_position, function() {
		parent.feedback(parent.module, "ACUATOR HAS BEEN SENT COMMAND"+parent.model.acuator.sent_position);
	});
};

Sensor.prototype.initTemp = function() {    
  var parent = this;
  this.intervals.queues.temp = setInterval(function(){
	fs.readFile('/sys/class/hwmon/hwmon0/device/temp1_input', 'utf8', function (err,data) {
		if (err) { return console.log(err); }
		parent.model.temperature.cpu = data/1000;
		if(parent.debug) {
			console.log("temperature: " + parent.model.temperature.cpu  );	
		}
	});
   }, this.intervals.periods.temp);
};

Sensor.prototype.initSignalTracker = function() {
	var parent = this;
	clearInterval(this.intervals.queues.signal);
	console.log("INIT SIGNAL TRACKER!!");
	this.intervals.queues.signal = setInterval(function() {
		http.get("http://verizonbrv/srv/status?_="+Math.random(), function(res) {
			//console.log("RES status = "+res.statusCode);
			res.on("data", function(chunk) {
				try {
					var status = JSON.parse(chunk);
					parent.model.signal.strength = parseInt(status["statusData"]["statusBarRSSI"]);
					parent.model.signal.bars = parseInt(status["statusData"]["statusBarSignalBars"]);
					// console.log("RSSI = "+status["statusData"]["statusBarRSSI"]+"dBm");
					// console.log("BARS = "+status["statusData"]["statusBarSignalBars"]);
				} catch(e) {
					console.log("chunk error: ",e);
					parent.model.signal.strength = -1;
					parent.model.signal.bars = -1;
				}
				//console.log("BODY: " + chunk);
			});
		}).on('error', function (err) {
			clearInterval(parent.intervals.queues.signal);
			parent.feedback(parent.module, "COULD NOT FIND VERIZON ROUTER, STOPPING SIGNAL STRENGTH MONITOR!");
			console.log("COULD NOT FIND VERIZON ROUTER, STOPPING SIGNAL STRENGTH MONITOR!");
		});
	}, this.intervals.periods.signal);
};
Sensor.prototype.resume = function() {};
Sensor.prototype.halt = function() {};
module.exports = exports = Sensor;