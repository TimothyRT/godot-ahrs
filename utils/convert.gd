extends Node


func vec3_to_ndarray(vec3: Vector3) -> NDArray:
	return nd.array([vec3.x, vec3.y, vec3.z])


func ndarray_to_quaternion(ndarray: NDArray):
	return Quaternion(
		ndarray.get(1).to_float(),
		ndarray.get(2).to_float(),
		ndarray.get(3).to_float(),
		ndarray.get(0).to_float()
	)


func quaternion_to_ndarray(quaternion: Quaternion):
	return nd.array([
		quaternion.w,
		quaternion.x,
		quaternion.y,
		quaternion.z
	])
