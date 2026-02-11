extends Node3D
## An implementation of the Madgwick filter (used to estimate device orientation from sensor data)
##
## Based on the following Python implementation:
## https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/madgwick.py


var acc: NDArray
var mag: NDArray
var gyro: NDArray
var madgwick: Madgwick
var madgwick_init := false

var time_since_last_redraw := 0.0

var leaky_bucket := {
	"gyro_x": [],
	"gyro_y": [],
	"gyro_z": [],
	"acc_x": [],
	"acc_y": [],
	"acc_z": [],
	"mag_x": [],
	"mag_y": [],
	"mag_z": []
}


func _ready() -> void:
	madgwick = Madgwick.new(null, null, null, 17.0)
	SignalBus.client_sensor_retrieved.connect(_on_client_sensor_retrieved)
	%Camera3D.make_current()


func _process(_delta: float) -> void:
	if len(leaky_bucket["gyro_x"]) > 0:
		gyro = nd.array([
			leaky_bucket["gyro_x"].pop_front(),
			leaky_bucket["gyro_y"].pop_front(),
			leaky_bucket["gyro_z"].pop_front()
		])
		acc = nd.array([
			leaky_bucket["acc_x"].pop_front(),
			leaky_bucket["acc_y"].pop_front(),
			leaky_bucket["acc_z"].pop_front()
		])
		mag = nd.array([
			leaky_bucket["mag_x"].pop_front(),
			leaky_bucket["mag_y"].pop_front(),
			leaky_bucket["mag_z"].pop_front()
		])
		redraw()


func _on_client_sensor_retrieved(_data_dict: Dictionary) -> void:
	for key in leaky_bucket:
		leaky_bucket[key].append_array(_data_dict[key])


func redraw() -> void:
	if gyro == null or acc == null or mag == null:
		return
	if madgwick_init == false:
		madgwick.Q = Convert.quaternion_to_ndarray(Quaternion.IDENTITY)
		madgwick_init = true
		return
	madgwick.Q = madgwick.update_MARG(madgwick.Q, gyro, acc, mag, 0.016)
	
	var res = Convert.ndarray_to_quaternion(madgwick.Q)
	%Capsule.quaternion = Quaternion(res.x, res.z, -res.y, res.w).normalized()
	print(%Capsule.quaternion)
	time_since_last_redraw = 0.0
	
	#var tilt = Tilt.new()
	#var temp = tilt.estimate(acc, mag)
	#%Capsule.quaternion = temp
