/**
 * Computer Animation Algorithms - Soft Body Physics
 *
 * @author Pratith Kanagaraj <pxk5958@rit.edu>, 2017
 */

/* global numeric */
/* global THREE */
/* global CCapture */

var softbody = function() {

// Set the scene size.
const WIDTH = 640;
const HEIGHT = 480;

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

var keyframes = [];

var prevKeyframeIndex = -1;
var nextKeyframeIndex = 0;
var prevKeyframe, nextKeyframe;
var ended = false;

var scene, camera, renderer, controls, clock;
var cube;

class FEM {
	constructor(mesh) {
		this.m_mesh = mesh;
		this.m_deltaTime = 0.0001;
		this.m_stiffnessMatrix = this.assembleK(this.m_mesh.getElements(), 
												this.m_mesh.getNodes(), 
												0.5, 0.5);
		
		this.m_floatingNodeStiffnessMatrix = this.buildFloatingNodeStiffnessMatrix();
		
		var nodes = this.m_mesh.getNodes();
		var fixedNodes = this.m_mesh.getFixedNodes();
		this.m_xdim = (nodes.length - fixedNodes.length) * 3;
		
		var massValue = 0.01;
		this.m_lumpedMassMatrix = this.buildLumpedMassMatrix(this.m_xdim, massValue);
		
		var x0 = numeric.rep([this.m_xdim, 1], 0);
		for (var i = 0; i < nodes.length; ++i) {
			var targetNodeIdx = this.m_originalNodeIndicesToNonFixed[i];
			if (targetNodeIdx == -1) {
				continue;
			}
			
			var targetIdx = targetNodeIdx * 3;
			
			var nonFixedNode = nodes[i];
			x0[targetIdx][0] = nonFixedNode.x;
			x0[targetIdx + 1][0] = nonFixedNode.y;
			x0[targetIdx + 2][0] = nonFixedNode.z;
		}
		this.m_x0 = x0;
		
		this.m_v = numeric.rep([this.m_xdim, 1], 0);
		
		this.m_x = numeric.clone(this.m_x0);
		
		this.m_A = numeric.add(this.m_lumpedMassMatrix, 
							numeric.mul(this.m_floatingNodeStiffnessMatrix, 
										this.m_deltaTime * this.m_deltaTime));
		
		this.m_force = numeric.rep([this.m_xdim, 1], 0);
	}
	
	update(deltaTime) {
		var displacement = numeric.sub(this.m_x, this.m_x0);
		
		var Kf = numeric.mul(this.m_floatingNodeStiffnessMatrix, displacement);
		
		var B = numeric.sub( numeric.mul(this.m_lumpedMassMatrix, this.m_v), 
				numeric.mul(numeric.sub(Kf, this.m_force), this.m_deltaTime) );
	
		this.preCGM(this.m_A, B, this.m_v, 30, 1e-5);
		numeric.addEq(this.m_x, numeric.mul(this.m_v, this.m_deltaTime));
		
		var nodes = this.m_mesh.getNodes();
		
		for (var i = 0; i < nodes.length; ++i) {
			var targetNodeIdx = this.m_originalNodeIndicesToNonFixed[i];
			if (targetNodeIdx == -1) {
				continue;
			}
			
			var targetIdx = targetNodeIdx * 3;
			
			var position = new THREE.Vector3(this.m_x[targetIdx][0], 
											this.m_x[targetIdx + 1][0], 
											this.m_x[targetIdx + 2][0]);
			
			this.m_mesh.setNodePosition(i, position);
		}
	}
	
	setForce(nodeIndex, force) {
		this.m_force = numeric.rep([this.m_xdim, 1], 0);
		
		var resultingIdx = this.m_originalNodeIndicesToNonFixed[nodeIndex];
		if (resultingIdx == -1) {
			return;
		}
		
		var forceIdx = resultingIdx * 3;
		this.m_force[forceIdx][0] = force.x;
		this.m_force[forceIdx + 1][0] = force.y;
		this.m_force[forceIdx + 2][0] = force.z;
	}
	
	buildLumpedMassMatrix(numNodes, totalMass) {
		var mass = totalMass / numNodes;
		
		var mLumpedMassMatrix = numeric.mul(numeric.identity(numNodes), mass);
		
		return mLumpedMassMatrix;
	}
	
	buildDampingMatrix(numNodes, damping) {
		return numeric.mul(numeric.identity(numNodes), damping);
	}
	
	buildBarycentric(x1, x2, x3, x4) {
		var Pe = numeric.rep([4, 4], 1);
		for (var i = 1; i < 4; ++i) {
			Pe[i][0] = x1.getComponent(i-1);
			Pe[i][1] = x2.getComponent(i-1);
			Pe[i][2] = x3.getComponent(i-1);
			Pe[i][3] = x4.getComponent(i-1);
		}

		return numeric.inv(Pe);
	}
	
	buildKe(x1, x2, x3, x4, mu, lambda) {
		var sigma, tr, vol;
		
		var Pe = this.buildBarycentric(x1, x2, x3, x4);
		vol = 1.0 / (numeric.det(Pe) * 6);
		
		var Ke = numeric.rep([12, 12], 0);
		                 
		for (var i = 0; i < 4; ++i) {
			for (var j = 0; j < 4; ++j) {
				var i3 = 3*i-1, j3 = 3*j-1;
				
				for (var a = 1; a < 4; ++a) {
					for (var b = 1; b < 4; ++b) {
						Ke[i3+a][j3+b] = 0.5 * vol * 
							(lambda*Pe[i][a]*Pe[j][b] + mu*Pe[i][b]*Pe[j][a]);
					}
				}
			}
			
			++i3;
			++j3;
			
			sigma = Pe[i][1] * Pe[j][1] + 
					Pe[i][2] * Pe[j][2] + 
					Pe[i][3] * Pe[j][3];
					
			tr = mu*0.5*vol*sigma;
			Ke[i3][j3] += tr;
			Ke[i3+1][j3+1] += tr;
			Ke[i3+2][j3+2] += tr;
		}
		
		return Ke;
	}
	
	assembleK(tList, vList, mu, lambda) {
		var vSize = vList.length;
		var tSize = tList.length;
		var K = numeric.rep([3*vSize, 3*vSize], 0);
		
		for (var tIdx = 0; tIdx < tSize; ++tSize) {
			var pIdx = numeric.rep([4], 0);
			pIdx[0] = tList[tIdx].p1Idx;
			pIdx[1] = tList[tIdx].p2Idx;
			pIdx[2] = tList[tIdx].p3Idx;
			pIdx[3] = tList[tIdx].p4Idx;
			
			var Ke = this.buildKe(vList[pIdx[0]],
								  vList[pIdx[1]],
								  vList[pIdx[2]],
								  vList[pIdx[3]],
								  mu, lambda);
								  
			for (var i = 0; i < 4; ++i) {
				for (var j = 0; j < 4; ++j) {
					var destI = pIdx[i]*3;
					var destJ = pIdx[j]*3;
					
					for (var t = 0; t < 3; ++t) {
						for (var m = 0; m < 3; ++m) {
							K[destI+t][destJ+m] += Ke[i*3+t][j*3+m];
						}
					}
				}
			}
		}
		
		return K;
	}
	
	buildFloatingNodeStiffnessMatrix() {
		// TODO
	}
	
	preCGM(S, b, x, iter, epsilon) {
		var r = numeric.rep(numeric.dim(b), 0);
		var d = numeric.rep(numeric.dim(b), 0);
		var q = numeric.rep(numeric.dim(b), 0);
		var s = numeric.rep(numeric.dim(b), 0);
		var sigmaOld, sigmaNew, alfa, beta;		
		
		// Initialization
		r = numeric.sub(b, numeric.mul(S, x));
		
		// The preconditioner solves: Pd = r
		this.jacobiPreSolve(S, r, d);
		
		sigmaNew = numeric.dot(r, d);
		
		for (var i = 0; i < iter && epsilon < sigmaNew; ++i) {
			q = numeric.mul(S, d);
			alfa = sigmaNew / numeric.dot(d, q);
			
			// Solution refinement
			numeric.addEq(x, numeric.mul(d, alfa));
			numeric.subeq(r, numeric.mul(q, alfa));
			
			// The preconditioner solves: Ps = r
			this.jacobiPreSolve(S, r, s);
			
			sigmaOld = sigmaNew;
			sigmaNew = numeric.dot(r, s);
			
			beta = sigmaNew / sigmaOld;
			d = numeric.add(s, numeric.mul(d, beta));
		}
	}
	
	jacobiPreSolve(P, b, x) {
		for (var k = 0; k < numeric.dim(b)[0]; ++k) {
			x[k][0] = b[k][0] / P[k][k];
		}
	}
}

function preinit() {
	loadTXT('keyframe-input.txt').then(function(data) {
		// get keyframe data
		
        var lines = data.split('\n');
        
        for (var i = 0; i < lines.length; ++i) {
        	var vals = [];
        	
        	var words = lines[i].split(/\s+/);
        	for (var j = 0; j < words.length; ++j) {
        		vals.push(parseFloat(words[j]));
        	}
        	
        	keyframes.push(vals);
        }
        nextKeyframe = keyframes[0];
        
        init();
        animate();
	});
}

/**
 * Initializes WebGL using three.js and sets up the scene
 */
function init() {
	// Create a WebGL scene
	scene = new THREE.Scene();
	
	// Start the renderer
	renderer = new THREE.WebGLRenderer();
	renderer.setSize( WIDTH, HEIGHT );

	// Add the render window to the document
    var container = document.getElementById( 'canvas' );
	container.appendChild( renderer.domElement );
	
	// Create a camera
	camera =
	    new THREE.PerspectiveCamera(
	        VIEW_ANGLE,
	        WIDTH/HEIGHT,
	        NEAR,
	        FAR
	    );
	camera.position.set(0, 0, -50);    
	
	// Add the camera to the scene.
	scene.add(camera);
	
	// Create controls
	controls = new THREE.TrackballControls( camera, renderer.domElement );
	controls.rotateSpeed = ROTATE;
	controls.zoomSpeed = ZOOM;
	controls.panSpeed = PAN;
	controls.noZoom = false;
	controls.noPan = false;
	controls.staticMoving = true;
	controls.dynamicDampingFactor = 0.3;
	controls.addEventListener( 'change', render );
	resetControls();
	
	clock = new THREE.Clock( true );
	
    // world

	var cubeMaterials = [];
	cubeMaterials.push(new THREE.MeshPhongMaterial( 
						{ color: 0xff0000, wireframe: false } ));
	cubeMaterials.push(new THREE.MeshPhongMaterial( 
						{ color: 0x00ff00, wireframe: false } ));
	cubeMaterials.push(new THREE.MeshPhongMaterial( 
						{ color: 0x0000ff, wireframe: false } ));
	cubeMaterials.push(new THREE.MeshPhongMaterial( 
						{ color: 0xffff00, wireframe: false } ));
	cubeMaterials.push(new THREE.MeshPhongMaterial( 
						{ color: 0x00ffff, wireframe: false } ));
	cubeMaterials.push(new THREE.MeshPhongMaterial( 
						{ color: 0xff00ff, wireframe: false } ));
    cube = new THREE.Mesh(
		new THREE.BoxGeometry(3, 3, 3),
		cubeMaterials
		);
	cube.position.set(0, 0, 0);
	scene.add(cube);
	
	// lights

	var light = new THREE.AmbientLight( 0xAAAAAA );
	scene.add( light );
	
	// Start capturer
	// capturer.start();
}

function loadTXT(filename) {
    function getText() {
        return new Promise( function( resolve, reject ) {
            var loader = new THREE.FileLoader();

			//load a text file a output the result to the console
			loader.load(
				filename,
			
			    // Function when resource is loaded
			    function ( data ) {
			        resolve(data);
			    }
			);
        });
    }
    
    return getText();
}

/**
 * Animates the scene
 */
function animate() {
	// Schedule the next frame.
	requestAnimationFrame(animate);
	
	var elapsed = clock.getElapsedTime();

	if (!ended && elapsed >= nextKeyframe[0]) {
		++prevKeyframeIndex;
		++nextKeyframeIndex;
		prevKeyframe = nextKeyframe;
		if (nextKeyframeIndex < keyframes.length) {
			nextKeyframe = keyframes[nextKeyframeIndex];
		} else {
			ended = true;
		}
	}
	
	if (!ended && prevKeyframeIndex >= 0) {
		var u = (elapsed - prevKeyframe[0]) 
				/ (nextKeyframe[0] - prevKeyframe[0]);
		
		var v1 = new THREE.Vector3(prevKeyframe[1], 
									prevKeyframe[2], 
									prevKeyframe[3]);
		var v2 = new THREE.Vector3(nextKeyframe[1], 
									nextKeyframe[2], 
									nextKeyframe[3]);
		
		// linear interpolation of position vectors
		
		var pos = new THREE.Vector3().addVectors(
			v1.multiplyScalar(1-u), v2.multiplyScalar(u));
		cube.position.set(pos.x, pos.y, pos.z);
		
		
		var q1 = new THREE.Quaternion();
		q1.setFromAxisAngle( new THREE.Vector3(prevKeyframe[4], 
								prevKeyframe[5], 
								prevKeyframe[6]), 
								THREE.Math.degToRad(prevKeyframe[7]) );
		q1.normalize();
		
		var q2 = new THREE.Quaternion();
		q2.setFromAxisAngle( new THREE.Vector3(nextKeyframe[4], 
								nextKeyframe[5], 
								nextKeyframe[6]), 
								THREE.Math.degToRad(nextKeyframe[7]) );
		q2.normalize();
		
		// spherical interpolation of quaternions
		
		THREE.Quaternion.slerp(q1, q2, cube.quaternion, u);
		cube.quaternion.normalize();
	}
	
	controls.update();

	render();
}

/**
 * Renders the scene
 */
function render() {
	renderer.render( scene, camera );
	
	// capturer.capture( renderer.domElement );
}

/**
 * Resets the controls (and camera) to default state
 */
function resetControls() {
	controls.reset();
	controls.target.set(0, 0, 100);
}

return {
	preinit: preinit,
	init: init,
	animate: animate,
	resetControls: resetControls
};

}();

softbody.preinit();