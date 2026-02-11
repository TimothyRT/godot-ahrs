extends Node
class_name Tilt


var acc: Variant
var mag: Variant
var angles: Variant


func _init(_acc = null, _mag = null) -> void:
	acc = _acc
	mag = _mag
	angles = null


#func _compute_all() -> NDArray:
	#if acc.ndim < 2:
		#return estimate(acc, mag)


func estimate(_acc: NDArray, _mag: NDArray) -> Quaternion:
	var a_norm = ndf.norm(_acc)
	var ax_ay_az = nd.divide(_acc, a_norm)
	var ax = ax_ay_az.get(0)
	var ay = ax_ay_az.get(1)
	var az = ax_ay_az.get(2)
	
	# Tilt from Accelerometer
	var ex = nd.atan2(ay, az)  # roll
	var ey = nd.atan2(nd.negative(ax), nd.sqrt(nd.add(nd.square(ay), nd.square(az))))  # pitch
	var ez = 0.0  # yaw
	
	if _mag != null:
		var m_norm = ndf.norm(_mag)
		var mx_my_mz = nd.divide(_mag, m_norm)
		var mx = mx_my_mz.get(0)
		var my = mx_my_mz.get(1)
		var mz = mx_my_mz.get(2)
		
		# Get tilted reference frame
		var by = nd.subtract(nd.multiply(my, nd.cos(ex)), nd.multiply(mz, nd.sin(ex)))
		var _tmp = nd.add(nd.multiply(my, nd.sin(ex)), nd.multiply(mz, nd.cos(ex)))
		var bx = nd.add(nd.multiply(mx, nd.cos(ey)), nd.multiply(nd.sin(ey), _tmp))
		
		ez = nd.atan2(nd.negative(by), bx)
	
	var cp = nd.cos(nd.multiply(0.5, ey))
	var sp = nd.sin(nd.multiply(0.5, ey))
	var cr = nd.cos(nd.multiply(0.5, ex))
	var sr = nd.sin(nd.multiply(0.5, ex))
	var cy = nd.cos(nd.multiply(0.5, ez))
	var sy = nd.sin(nd.multiply(0.5, ez))
	
	var w = nd.add(nd.multiply(cy, nd.multiply(cp, cr)), nd.multiply(sy, nd.multiply(sp, sr)))
	var x = nd.subtract(nd.multiply(cy, nd.multiply(cp, sr)), nd.multiply(sy, nd.multiply(sp, cr)))
	var y = nd.add(nd.multiply(sy, nd.multiply(cp, sr)), nd.multiply(cy, nd.multiply(sp, cr)))
	var z = nd.subtract(nd.multiply(sy, nd.multiply(cp, cr)), nd.multiply(cy, nd.multiply(sp, sr)))
	
	var q = Quaternion(x.to_float(), z.to_float(), -y.to_float(), w.to_float())
	return q
