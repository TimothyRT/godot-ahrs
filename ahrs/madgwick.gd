extends Node
class_name Madgwick


var gyr: Variant
var acc: Variant
var mag: Variant
var frequency: float
var Dt: float
var q0: NDArray
var gain_imu: float
var gain_marg: float
var gain: float
var Q: NDArray


func _init(
	_gyr=null,
	_acc=null,
	_mag=null,
	_frequency=100.0,
	_Dt=0.0,
	_q0=nd.array([0.7071, 0.0, 0.7071, 0.0]),
	_gain_imu=0.033,
	_gain_marg=0.041
	) -> void:
	if _gyr != null:
		gyr = _gyr
	if _acc != null:
		acc = _acc
	if _mag != null:
		mag = _mag
	
	if _frequency != 0.0:
		frequency = _frequency
	else:
		frequency = 100.0
	
	if _Dt == 0.0:
		_Dt = 1.0/frequency
	else:
		Dt = _Dt
	
	q0 = _q0
	
	gain_imu = _gain_imu
	gain_marg = _gain_marg
	gain = gain_marg
	
	if acc != null and gyr != null:
		Q = _compute_all()


func _compute_all() -> NDArray:
	gyr = nd.copy(gyr)
	acc = nd.copy(acc)
	if acc.shape != gyr.shape:
		push_error("acc and gyr are not the same size")
	var num_samples = len(acc)
	var _Q = nd.zeros([num_samples, 4])
	
	# Compute with IMU architecture
	if mag == null:
		_Q.set(nd.divide(q0, nd.norm(self.q0)), 0)
		for t in range(1, num_samples):
			_Q.set(update_IMU(Q.get(t - 1), gyr.get(t), acc.get(t)), t)
		return _Q
	
	# Compute with MARG architecture
	mag = nd.copy(mag)
	if mag.shape != gyr.shape:
		push_error("mag and gyr are not the same size")
	_Q.set(ecompass(acc.get(0), mag.get(0)), 0)
	for t in range(1, num_samples):
		_Q.set(update_MARG(Q.get(t - 1), gyr.get(t), acc.get(t), mag.get(t)), t)
	return _Q


func shepperd(dcm: NDArray) -> NDArray:
	var r11 = dcm.get(0).get(0).to_float()
	var r12 = dcm.get(0).get(1).to_float()
	var r13 = dcm.get(0).get(2).to_float()
	var r21 = dcm.get(1).get(0).to_float()
	var r22 = dcm.get(1).get(1).to_float()
	var r23 = dcm.get(1).get(2).to_float()
	var r31 = dcm.get(2).get(0).to_float()
	var r32 = dcm.get(2).get(1).to_float()
	var r33 = dcm.get(2).get(2).to_float()
	
	var u = nd.array([r11+r22+r33, r11, r22, r33])
	var u0 = u.get(0).to_float()
	var u1 = u.get(1).to_float()
	var u2 = u.get(2).to_float()
	var u3 = u.get(2).to_float()
	
	var d: float
	var q: NDArray
	if u0 >= u1 and u0 >= u2 and u0 >= u3:
		d = sqrt(1.0+r11+r22+r33)
		q = nd.multiply(0.5, nd.array([d, (r32-r23)/d, (r13-r31)/d, (r21-r12)/d]))
	elif u1 >= u0 and u1 >= u2 and u1 >= u3:
		d = sqrt(1.0+r11-r22-r33)
		q = nd.multiply(0.5, nd.array([(r32-r23)/d, d, (r12+r21)/d, (r31+r13)/d]))
	elif u2 >= u0 and u2 >= u1 and u2 >= u3:
		d = sqrt(1.0-r11+r22-r33)
		q = nd.multiply(0.5, nd.array([(r13-r31)/d, (r12+r21)/d, d, (r23+r32)/d]))
	else:
		d = sqrt(1.0-r11-r22+r33)
		q = nd.multiply(0.5, nd.array([(r21-r12)/d, (r31+r13)/d, (r32+r23)/d, d]))
	q = nd.divide(q, nd.norm(q))
	return q


func ecompass(_a: NDArray, _m: NDArray) -> NDArray:
	var a = nd.copy(_a)
	var m = nd.copy(_m)
	if a.shape != m.shape:
		push_error("Both vectors must have the same shape.")
	m = nd.divide(m, nd.norm(m))
	var Rz = nd.divide(a, nd.norm(a))
	# ENU frame
	var Rx = nd.cross(m, Rz)
	var Ry = nd.cross(Rz, Rx)
	Rx = nd.divide(Rx, nd.norm(Rx)) 
	Ry = nd.divide(Ry, nd.norm(Ry))
	var R = nd.reshape(nd.concatenate([Rx, Ry, Rz], 0), [3, 3])
	return shepperd(R)


func update_IMU(_q: NDArray, _gyr: NDArray, _acc: NDArray, dt=0.0) -> NDArray:
	var q = Convert.ndarray_to_quaternion(_q)
	if dt == 0.0:
		dt = Dt
	var qDot = 0.5 * (q * Quaternion(_gyr.get(0).to_float(), _gyr.get(1).to_float(), _gyr.get(2).to_float(), 0.0))
	var a_norm = nd.norm(_acc)
	if a_norm.to_float() > 0:
		var a = nd.divide(_acc, a_norm)
		var qx_qy_qz_qw = q.normalized()
		var qx = qx_qy_qz_qw.x
		var qy = qx_qy_qz_qw.y
		var qz = qx_qy_qz_qw.z
		var qw = qx_qy_qz_qw.w
		
		# Objective function (eq. 25)
		var f := nd.array([
			2.0*(qx*qz - qw*qy)   - a.get(0),
			2.0*(qw*qx + qy*qz)   - a.get(1),
			2.0*(0.5-qx**2-qy**2) - a.get(2)
		])
		if nd.norm(f).to_float() > 0:
			var J = nd.array([
				[-2.0*qy,  2.0*qz, -2.0*qw, 2.0*qx],
				[ 2.0*qx,  2.0*qw,  2.0*qz, 2.0*qy],
				[ 0.0,    -4.0*qx, -4.0*qy, 0.0   ]
			])
			# Objective Function Gradient
			var gradient = nd.matmul(nd.transpose(J), f)
			gradient = nd.divide(gradient, nd.norm(gradient))
			qDot -= self.gain*gradient
	var q_new: Quaternion = q + qDot * dt
	q_new = q_new / nd.norm(q_new).to_float()
	return Convert.quaternion_to_ndarray(q_new)


func update_MARG(_q: NDArray, _gyr: NDArray, _acc: NDArray, _mag: NDArray, dt=0.0) -> NDArray:
	var q: Quaternion = Convert.ndarray_to_quaternion(_q)
	if dt == 0.0:
		dt = Dt
	if nd.norm(_gyr).to_float() == 0:
		return _q
	if nd.norm(_mag).to_float() == 0:
		return update_IMU(_q, _gyr, _acc)
	q = q.normalized()
	var qDot = 0.5 * (q * Quaternion(_gyr.get(0).to_float(), _gyr.get(1).to_float(), _gyr.get(2).to_float(), 0.0))
	var a_norm = nd.norm(_acc)
	if a_norm.to_float() > 0:
		var a = nd.divide(_acc, a_norm)
		var m = nd.divide(_mag, nd.norm(_mag))
		# Rotate normalized magnetometer measurements
		var q_m = Quaternion(m.get(0).to_float(), m.get(1).to_float(), m.get(2).to_float(), 0.0)
		var h = q * (q_m * q.inverse())
		var bx = nd.norm(nd.array([h.x, h.y])).to_float()
		var bz = h.z
		var qx_qy_qz_qw = q.normalized()
		var qx = qx_qy_qz_qw.x
		var qy = qx_qy_qz_qw.y
		var qz = qx_qy_qz_qw.z
		var qw = qx_qy_qz_qw.w
		# Objective function (eq. 31)
		var f = nd.array([
			2.0*(qx*qz - qw*qy)   - a.get(0).to_float(),
			2.0*(qw*qx + qy*qz)   - a.get(1).to_float(),
			2.0*(0.5-qx**2-qy**2) - a.get(2).to_float(),
			2.0*bx*(0.5 - qy**2 - qz**2) + 2.0*bz*(qx*qz - qw*qy)       - m.get(0).to_float(),
			2.0*bx*(qx*qy - qw*qz)       + 2.0*bz*(qw*qx + qy*qz)       - m.get(1).to_float(),
			2.0*bx*(qw*qy + qx*qz)       + 2.0*bz*(0.5 - qx**2 - qy**2) - m.get(2).to_float()
			])
		if nd.norm(f).to_float() > 0:
			# Jacobian (eq. 32)
			var J = nd.array([
				[-2.0*qy,               2.0*qz,              -2.0*qw,               2.0*qx             ],
				[ 2.0*qx,               2.0*qw,               2.0*qz,               2.0*qy             ],
				[ 0.0,                 -4.0*qx,              -4.0*qy,               0.0                ],
				[-2.0*bz*qy,            2.0*bz*qz,           -4.0*bx*qy-2.0*bz*qw, -4.0*bx*qz+2.0*bz*qx],
				[-2.0*bx*qz+2.0*bz*qx,  2.0*bx*qy+2.0*bz*qw,  2.0*bx*qx+2.0*bz*qz, -2.0*bx*qw+2.0*bz*qy],
				[ 2.0*bx*qy,            2.0*bx*qz-4.0*bz*qx,  2.0*bx*qw-4.0*bz*qy,  2.0*bx*qx          ]])
			var gradient = nd.matmul(nd.transpose(J), f)
			gradient = nd.divide(gradient, nd.norm(gradient))
			var gradient_reordered = nd.array([gradient.get(1), gradient.get(2), gradient.get(3), gradient.get(0)])
			qDot = Convert.ndarray_to_quaternion(nd.subtract(Convert.quaternion_to_ndarray(qDot), nd.multiply(gain, gradient_reordered)))
	var q_new: Quaternion = q + qDot * dt
	q_new = q_new.normalized()
	return Convert.quaternion_to_ndarray(q_new)
