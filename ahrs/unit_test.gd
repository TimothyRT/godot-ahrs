extends Node


func _ready() -> void:
	var gyro_data = nd.array([3.0, -13.0, 3.2])
	var accel_data = nd.array([4.2, 5.0, -4.0])
	var mag_data = nd.array([0.3, -4.0, 2.0])
	var madgwick = Madgwick.new(null, null, null, 17.0)
	madgwick.Q = Convert.quaternion_to_ndarray(Quaternion.IDENTITY)
	print(madgwick.Q)
	var t0 = Time.get_ticks_usec()
	var new_q = madgwick.update_MARG(madgwick.Q, gyro_data, accel_data, mag_data, 1/17.0)
	var t1 := Time.get_ticks_usec()
	var elapsed_us = t1 - t0
	print("AHRS update:", elapsed_us, "µs")
	#print(new_q)
	
	var res = Convert.ndarray_to_quaternion(new_q)
	var res_q = Quaternion(res.x, res.z, -res.y, res.w)
	print(res_q)
	#print(res_q.x)
	#print(res_q.y)
	#print(res_q.z)
	#print(res_q.w)
