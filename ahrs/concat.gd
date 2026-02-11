extends Node


func _ready() -> void:
	var Rx = nd.array([-0.895189, 0.444371, -0.034232])
	var Ry = nd.array([0.445413, 0.894691, -0.033695])
	var Rz = nd.array([0.015654, -0.045411, -0.998846])
	var R = nd.concatenate([Rx, Ry, Rz], 0)
	print("R: ", R)
