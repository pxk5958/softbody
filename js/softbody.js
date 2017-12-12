/**
 * Computer Animation Algorithms - Soft Body Physics
 *
 * @author Pratith Kanagaraj <pxk5958@rit.edu>, 2017
 */

const FPS = 60;
const N_POINTS = 30;
const N_SPRINGS = N_POINTS;
const LENGTH = 75;
const R = 190.0;
const R2 = R * R;
const BALL_R = 0.516;
const CUBE_S = 1;
const GY = 110.0;
const FAPP = 110.0;
const DT = 0.01;
const MIN_PRESSURE = 3000;
const DEF_PRESSURE = 70000;
const MAX_PRESSURE = 2500000;
const MIN_MASS = 1.0;
const DEF_MASS = 1.0;
const MAX_MASS = 5.0;
const MIN_KS = 17.0;
const DEF_KS = 755.0;
const MAX_KS = 2000.0;
const MIN_KD = 0.0;
const DEF_KD = 40.0;
const MAX_KD = 70.0;
//Tangential and normal damping factors
const TDF = 0.99;                         //0.95 by default, 1.0 works, 1.01 is cool
// A TDF of 1.0 means frictionless boundaries.
// If some energy were not lost due to the ball's
// spring-damping, the ball could continue
// traveling forever without any force.
const NDF = 0.1;

// Set the scene size.
const WIDTH = 800;
const HEIGHT = 500;

// Set some camera attributes.
const VIEW_ANGLE = 45;
const NEAR = 0.1;
const FAR = 10000;

// Set the controls speeds
const ROTATE = 15;
const ZOOM = 10;
const PAN = 6;

// Create a capturer that exports a WebM video
// var capturer  = new CCapture( { 
// 	format: 'webm',
// 	timeLimit: 10
// } );

var pressure = DEF_PRESSURE;
var points, springs;
var upArrow = false, downArrow = false, leftArrow = false, rightArrow = false;
var mouseP = false;

var pressureSlider;
var massSlider;
var ksSlider;
var kdSlider;

var options = {
	maxPressure: DEF_PRESSURE,
	mass: DEF_MASS,
	ks: DEF_KS,
	kd: DEF_KD
};

/**

 * Initializes WebGL using three.js and sets up the scene
 */
function setup() {
	createCanvas(WIDTH + 400, HEIGHT + 100);
    frameRate(FPS);
    
    // UI
    pressureSlider = createSlider(MIN_PRESSURE, MAX_PRESSURE, DEF_PRESSURE);
	pressureSlider.position(WIDTH + 20, 50);
	
	massSlider = createSlider(MIN_MASS, MAX_MASS, DEF_MASS);
	massSlider.position(WIDTH + 20, 100);
	
	ksSlider = createSlider(MIN_KS, MAX_KS, DEF_KS);
	ksSlider.position(WIDTH + 20, 150);
	
	//kdSlider = createSlider(MIN_KD, MAX_KD, DEF_KD);
	//kdSlider.position(WIDTH + 20, 200);
    
    // world
	createBall();
}

function draw() {
	background(255);
	
	fill(0);
	text("Pressure", pressureSlider.x + pressureSlider.width + 20, 50);
	text("Mass", massSlider.x + massSlider.width + 20, 100);
	text("ks (spring constant)", ksSlider.x + ksSlider.width + 20, 150);
	//text("kd (damping factor)", kdSlider.x + kdSlider.width + 20, 200);
	
	fill(255);
	//ellipse(WIDTH/2, HEIGHT/2, 2*R, 2*R);
	rect(0, 0, WIDTH, HEIGHT);
	
	fill('#ff0000');
	noStroke();
	beginShape();
	for (var i=0; i < points.x.length; i++) {
		vertex(points.x[i], points.y[i]);
	}
	endShape(CLOSE);

	options.maxPressure = pressureSlider.value();
	options.mass = massSlider.value();
	options.ks = ksSlider.value();
	options.kd = DEF_KD;
	update();
	update(); // get 2 bits of work done for 1 frame
	// better than just increasing DT, since we still converge
	
	stroke(0);
	if (mouseP) {
		line(mouseX, mouseY, points.x[0], points.y[0]);
	}
}


function keyPressed() {
	switch (keyCode) {
		case UP_ARROW: upArrow=true; break;
		case DOWN_ARROW: downArrow=true; break;
		case LEFT_ARROW: leftArrow=true; break;
		case RIGHT_ARROW: rightArrow=true; break;
		default: break;
	}
}

function keyReleased() {
	switch (keyCode) {
		case UP_ARROW: upArrow=false; break;
		case DOWN_ARROW: downArrow=false; break;
		case LEFT_ARROW: leftArrow=false; break;
		case RIGHT_ARROW: rightArrow=false; break;
		default: break;
	}
}

function mousePressed() {
	mouseP = (mouseX >= 0 && mouseX <= WIDTH && mouseY >= 0 && mouseY <= HEIGHT);
}

function mouseReleased() {
	mouseP = false;
}


function addSpring(i, j, k) {
	springs.spring1[i] = j;
	springs.spring2[i] = k;
	
	springs.length[i] = Math.sqrt( 
		(points.x[j] - points.x[k])*(points.x[j] - points.x[k]) 
		+ (points.y[j] - points.y[k])*(points.y[j] - points.y[k]) 
	);
}

function createBall() {
	points = new Points(N_POINTS);
	springs = new Springs(N_SPRINGS);
	
	for (var i = 0; i < N_POINTS; ++i) {
		points.x[i] = BALL_R * Math.cos(i * 2 * Math.PI / N_POINTS) + WIDTH/2;
		points.y[i] = BALL_R * Math.sin(i * 2 * Math.PI / N_POINTS) + HEIGHT/4;
	}
	
	for (var i = 0; i < N_POINTS-1; ++i) {
		addSpring(i, i, i+1);
	}
	addSpring(N_POINTS-1, N_POINTS-1, 0);
}

function createCube() {
	points = new Points(N_POINTS);
	springs = new Springs(N_SPRINGS);
	
	for (var i = 0; i < N_POINTS/4; ++i) {
		points.x[0 + i] = WIDTH/2 + i*CUBE_S/((N_POINTS/4) * 1.0);
		points.y[0 + i] = HEIGHT/2;
	}
	for (var i = 0; i < N_POINTS/4; ++i) {
		points.x[N_POINTS/4 + i] = WIDTH/2 + CUBE_S;
		points.y[N_POINTS/4 + i] = HEIGHT/2 - i*CUBE_S/((N_POINTS/4) * 1.0);
	}
	for (var i = 0; i < N_POINTS/4; ++i) {
		points.x[N_POINTS/2 + i] = WIDTH/2 + CUBE_S - i*CUBE_S/((N_POINTS/4) * 1.0);
		points.y[N_POINTS/2 + i] = HEIGHT/2 - CUBE_S;
	}
	for (var i = 0; i < N_POINTS/4; ++i) {
		points.x[N_POINTS*3/4 + i] = WIDTH/2;
		points.y[N_POINTS*3/4 + i] = HEIGHT/2 - CUBE_S + i*CUBE_S/((N_POINTS/4) * 1.0);
	}
	
	for (var i = 0; i < N_POINTS-1; ++i) {
		addSpring(i, i, i+1);
	}
	addSpring(N_POINTS-1, N_POINTS-1, 0);
}

function accumulateForces() {
	var x1, x2, y1, y2;
	var r12d;
	var vx12;
	var vy12;
	var f;
	var fx0, fy0;
	var volume = 0;
	var pressurev;
	
	for (var i = 0; i < N_POINTS; ++i) {
		points.fx[i] = 0;
		points.fy[i] = (pressure - options.maxPressure) >= 0 ? GY*options.mass : 0;
		
		if (upArrow) {
			points.fy[i] = -FAPP*options.mass;
		}
		if (rightArrow) {
			points.fx[i] = FAPP*options.mass;
		}
		if (leftArrow) {
			points.fx[i] = -FAPP*options.mass;
		}
		if (downArrow) {
			points.fy[i] = FAPP*options.mass;
		}
		if (leftArrow && rightArrow) {
			points.fx[i] = 0.0;
		}
		if (upArrow && downArrow) {
			points.fy[i] = 0.0;
		}
	}
	
	if (mouseP) {
		x1 = points.x[0];
		y1 = points.y[0];
		x2 = mouseX;
		y2 = mouseY;
		
		r12d = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
		f = (r12d - 2.2) * 22 
			+ (points.vx[0] * (x1 - x2) + points.vy[0] * (y1 - y2)) * 54 / r12d;

		fx0 = ((x1 - x2) / r12d ) * f;
		fy0 = ((y1 - y2) / r12d ) * f;

		points.fx[0] -= fx0;
		points.fy[0] -= fy0;
	}
	
	for (var i = 0; i < N_SPRINGS; i++) {
		x1 = points.x[springs.spring1[i]];
		x2 = points.x[springs.spring2[i]];
		y1 = points.y[springs.spring1[i]];
		y2 = points.y[springs.spring2[i]];

		// Find the distance between each spring:
		r12d = Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

		// Accumulate spring forces:
		if (r12d != 0) {
			vx12 = points.vx[springs.spring1[i]] - points.vx[springs.spring2[i]];
			vy12 = points.vy[springs.spring1[i]] - points.vy[springs.spring2[i]];

			f = (r12d - springs.length[i]) * options.ks + (vx12 * (x1 - x2) + vy12 * (y1 - y2)) * options.kd / r12d;

			fx0 = ((x1 - x2) / r12d ) * f;
			fy0 = ((y1 - y2) / r12d ) * f;

			points.fx[springs.spring1[i]] -= fx0;
			points.fy[springs.spring1[i]] -= fy0;

			points.fx[springs.spring2[i]] += fx0;
			points.fy[springs.spring2[i]] += fy0;
		}
		
		// Calculate normal vectors for use with finding pressure force:
		springs.nx[i] = -(y1 - y2) / r12d;
		springs.ny[i] = (x1 - x2) / r12d;
	}

	for (var i = 0; i < N_SPRINGS; i++) {
		x1 = points.x[springs.spring1[i]];
		x2 = points.x[springs.spring2[i]];
		y1 = points.y[springs.spring1[i]];
		y2 = points.y[springs.spring2[i]];

		r12d = Math.sqrt((x1 - x2) *(x1 - x2)  +  (y1 - y2) * (y1 - y2));

		volume += 0.5 * Math.abs(x1 - x2) 
			* Math.abs(springs.nx[i] != 0 ? springs.nx[i] : springs.ny[i]) 
			* (r12d);
	}
	
	for (var i = 0; i < N_SPRINGS; i++) {
		x1 = points.x[springs.spring1[i]];
		x2 = points.x[springs.spring2[i]];
		y1 = points.y[springs.spring1[i]];
		y2 = points.y[springs.spring2[i]];

		r12d = Math.sqrt((x1 - x2) * (x1 - x2)  +  (y1 - y2) * (y1 - y2));

		pressurev = r12d * pressure * (1.0/volume);

		points.fx[springs.spring1[i]] += springs.nx[i]*pressurev;
		points.fy[springs.spring1[i]] += springs.ny[i]*pressurev;
		points.fx[springs.spring2[i]] += springs.nx[i]*pressurev;
		points.fy[springs.spring2[i]] += springs.ny[i]*pressurev;
	}
}

function integrateHeun() {
	var drx, dry;
	
	var fxsaved = [], fysaved = [], vxsaved = [], vysaved = [];
	
	for (var i = 0; i < N_POINTS; ++i) {
		fxsaved.push(points.fx[i]);
		fysaved.push(points.fy[i]);
		
		vxsaved.push(points.vx[i]);
		vysaved.push(points.vy[i]);
		
		points.vx[i] += points.fx[i] / options.mass * DT;
		drx = points.vx[i] * DT;
		points.x[i] += drx;
		
		points.vy[i] += points.fy[i] / options.mass * DT;
		dry = points.vy[i] * DT;
		points.y[i] += dry;
	}
	
	accumulateForces();
	
	for (var i = 0; i < N_POINTS; ++i) {
		points.vx[i] = vxsaved[i] + (points.fx[i] + fxsaved[i]) / options.mass * DT/2;
		drx = points.vx[i] * DT;
		//points.x[i] += drx;
		
		points.vy[i] = vysaved[i] + (points.fy[i] + fysaved[i]) / options.mass * DT/2;
		dry = points.vy[i] * DT;
		//points.y[i] += dry;
		
		/*
		points.x[i] = Math.min(points.x[i], WIDTH/2.0 + R);
		points.x[i] = Math.max(points.x[i], WIDTH/2.0 - R);

		points.y[i] = Math.min(points.y[i], HEIGHT/2.0 + R);
		points.y[i] = Math.max(points.y[i], HEIGHT/2.0 - R);

		if (points.x[i] + drx > Math.sqrt(R2 - Math.pow(points.y[i] - HEIGHT/2.0, 2)) + WIDTH/2.0 
			|| points.x[i] + drx < -Math.sqrt(R2 - Math.pow(points.y[i] - HEIGHT/2.0, 2)) + WIDTH/2.0)
		{
			drx *= -1;                           //These are temporary until I do
			dry *= -1;                           //the math to get more exact values.

			var vx0 = points.vx[i];
			var vy0 = points.vy[i];

			var sinTheta = (points.y[i] - HEIGHT/2.0) / R;
			var cosTheta = (points.x[i] - WIDTH/2.0) / R;

			points.vx[i] = -vx0;
			points.vy[i] = -vy0;
			points.vx[i] = vy0 * (-TDF * sinTheta * cosTheta - NDF * sinTheta * cosTheta) + vx0 * (TDF * sinTheta * sinTheta - NDF * cosTheta * cosTheta);
			points.vy[i] = vy0 * (TDF * cosTheta * cosTheta - NDF * sinTheta * sinTheta) + vx0 * (-TDF * sinTheta * cosTheta - NDF * sinTheta * cosTheta);
		}

		if (points.y[i] > HEIGHT/2.0 + R/2.0) { // need these checks to avoid setting to wrong sign
			points.y[i] = Math.min(points.y[i], Math.sqrt(Math.abs(R2 - Math.pow(points.x[i] - WIDTH/2.0, 2))) + HEIGHT/2.0);
		}

		if (points.y[i] < HEIGHT/2.0 - R/2.0) {
			points.y[i] = Math.max(points.y[i], -Math.sqrt(Math.abs(R2 - Math.pow(points.x[i] - WIDTH/2.0, 2))) + HEIGHT/2.0);
		}

		if (points.x[i] > WIDTH/2.0 + R/2.0) {
			points.x[i] = Math.min(points.x[i], Math.sqrt(Math.abs(R2 - Math.pow(points.y[i] - HEIGHT/2.0, 2))) + WIDTH/2.0);
		}

		if (points.x[i] < WIDTH/2.0 - R/2.0) {
			points.x[i] = Math.max(points.x[i], -Math.sqrt(Math.abs(R2 - Math.pow(points.y[i] - HEIGHT/2.0, 2))) + WIDTH/2.0);
		}
		*/
		
		if (points.y[i] + dry < 0) {
			points.y[i] = 0;
			points.vy[i] = - 0.1 * points.vy[i];
		} else if (points.y[i] + dry > HEIGHT) {
			points.y[i] = HEIGHT;
			points.vy[i] = 0.1 * points.vy[i];
		} else {
			points.y[i] += dry;
		}
		
		if (points.x[i] + drx < 0) {
			points.x[i] = 0;
			points.vx[i] = - 0.1 * points.vx[i];
		} else if (points.x[i] + drx > WIDTH) {
			points.x[i] = WIDTH;
			points.vx[i] = 0.1 * points.vx[i];
		} else {
			points.x[i] += drx;
		}
		
		if (points.x[i] > WIDTH || points.y[i] > HEIGHT) {
			console.log('oops');
		}
	}
}

function update() {
	accumulateForces();
	integrateHeun();
	
	// if (pressure < options.maxPressure) {
	// 	pressure += options.maxPressure / 300.0;
	// }
	pressure = options.maxPressure;
}

class Points {
	constructor(n_pts) {
		this.x = [];
		this.y = [];
		this.vx = [];
		this.vy = [];
		this.fx = [];
		this.fy = [];
		
		for (var i = 0; i < n_pts; ++i) {
			this.x.push(0);
			this.y.push(0);
			this.vx.push(0);
			this.vy.push(0);
			this.fx.push(0);
			this.fy.push(0);
		}
	}
}

class Springs {
	constructor(n_springs) {
		this.spring1 = [];
		this.spring2 = [];
		this.length = [];
		this.nx = [];
		this.ny = [];
		
		for (var i = 0; i < n_springs; ++i) {
			this.spring1.push(0);
			this.spring2.push(0);
			this.length.push(0);
			this.nx.push(0);
			this.ny.push(0);
		}
	}
}