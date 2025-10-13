from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time

class WindowFinder(Node):
	def __init__(self):
		super().__init__('window_finder')

		self._centroid_publisher = self.create_publisher(Point, 'centroid', 10)
		self._threshold_publisher = self.create_publisher(UInt8, 'threshold', 10)
		self._window_found_publisher = self.create_publisher(Bool, 'window_found', 10)

		# Declare parameter for topic and message type
		self.declare_parameter('depth_topic', '/depth_camera/image_raw')
		self.declare_parameter('use_compressed', False)
		self.declare_parameter('scale', 0.5)
		self.declare_parameter('debug_fps', 4.0)
		self.declare_parameter('depth_min_m', 0.2)
		self.declare_parameter('depth_max_m', 10.0)
		self.declare_parameter('depth_unit_scale', 0.001)  # for 16UC1 mm -> meters

		depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
		use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
		self.scale = float(self.get_parameter('scale').get_parameter_value().double_value)
		debug_fps = float(self.get_parameter('debug_fps').get_parameter_value().double_value)
		self.depth_min = float(self.get_parameter('depth_min_m').get_parameter_value().double_value)
		self.depth_max = float(self.get_parameter('depth_max_m').get_parameter_value().double_value)
		self.depth_unit_scale = float(self.get_parameter('depth_unit_scale').get_parameter_value().double_value)
		self._debug_period = 1.0 / debug_fps if debug_fps > 0 else 0.0
		self._last_debug_pub_ts = 0.0

		if use_compressed:
			self._subscriber = self.create_subscription(
				CompressedImage,
				depth_topic,
				self.compressed_depth_callback,
				10
			)
			self.get_logger().info(f'Subscribed to COMPRESSED depth topic: {depth_topic}')
		else:
			self._subscriber = self.create_subscription(
				Image,
				depth_topic,
				self.raw_depth_callback,
				10
			)
			self.get_logger().info(f'Subscribed to RAW depth topic: {depth_topic}')

		self.bridge = CvBridge()

		self.threshold = UInt8()
		self.threshold.data = int(8) # valor do threshold

		# Publishers de debug para visualizar no rqt
		self._annotated_pub = self.create_publisher(Image, '/window_detector/annotated', 1)
		self._mask_pub = self.create_publisher(Image, '/window_detector/mask', 1)
	
	def raw_depth_callback(self, msg):
		"""Callback for raw (uncompressed) depth images from Gazebo"""
		try:
			enc = (msg.encoding or '').lower()
			if enc in ('32fc1', '32fc'):
				depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
			elif enc in ('16uc1', 'mono16'):
				d16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
				depth = d16.astype(np.float32) * self.depth_unit_scale
				depth[d16 == 0] = np.nan
			elif enc in ('mono8', 'rgb8', 'bgr8'):
				self.get_logger().warn(f'Depth topic has non-depth encoding "{msg.encoding}". Skipping frame.')
				return
			else:
				# Best effort: try 32FC1, then 16UC1
				try:
					depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
				except CvBridgeError:
					d16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
					depth = d16.astype(np.float32) * self.depth_unit_scale
					depth[d16 == 0] = np.nan
		except CvBridgeError as e:
			self.get_logger().error(f'CvBridge Error (raw): {e}')
			return

		self.process_depth_image(depth)
	
	def compressed_depth_callback(self, msg):
		"""Callback for compressed depth images (PNG format with 12-byte header)"""
		self.depth_image_callback(msg)
	
	def depth_image_callback(self, msg):
		'''
		Com o comando ros2 topic echo --once /depth_camera/compressedDepth podemos descobrir:

		1. A imagem original, antes de ser comprimida, era do tipo float32, ou seja: 32FC1. Isso significa que os valores 
			de pixel já representam a distância em metros.
		
		2. A sequência 137, 80, 78, 71 corresponde aos bytes \x89, P, N, G em decimal. Ou seja, é um PNG

		3. Existe um cabeçalho de 12 bytes

		'''

		png_data = msg.data[12:]
		np_arr = np.frombuffer(png_data, np.uint8)
		depth_meters = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

		if depth_meters is None:
			self.get_logger().error('Failed to decode compressed depth image (frame is None).')
			return
		
		self.process_depth_image(depth_meters)
	
	def process_depth_image(self, depth_meters):
		if depth_meters is None:
			return

		# Downscale
		if self.scale and self.scale != 1.0:
			depth_meters = cv2.resize(depth_meters, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_NEAREST)

		h, w = depth_meters.shape[:2]
		center_x = w // 2
		center_y = h // 2

		d = depth_meters.astype(np.float32).copy()
		d[~np.isfinite(d)] = np.nan

		valid = np.isfinite(d) & (d >= self.depth_min) & (d <= self.depth_max)
		valid_ratio = float(np.count_nonzero(valid)) / max(d.size, 1)

		# Normalize inverted for visualization, with fallback if no valid pixels
		if np.any(valid):
			span = max(self.depth_max - self.depth_min, 1e-6)
			clamped = np.clip(d, self.depth_min, self.depth_max)
			n = (1.0 - (clamped - self.depth_min) / span) * 255.0
			n = np.clip(n, 0, 255).astype(np.uint8)
			n[~np.isfinite(n)] = 0
		else:
			finite = np.isfinite(d)
			if np.any(finite):
				p1, p2 = np.nanpercentile(d[finite], [5, 95])
				if not np.isfinite(p1) or not np.isfinite(p2) or p2 <= p1:
					p1, p2 = np.nanmin(d[finite]), np.nanmax(d[finite])
				n = ((p2 - d) / max(p2 - p1, 1e-6)) * 255.0
				n = np.clip(n, 0, 255).astype(np.uint8)
				n[~finite] = 0
			else:
				n = np.zeros_like(d, dtype=np.uint8)

			now = time.time()
			if now - getattr(self, "_last_warn", 0.0) > 1.0:
				self.get_logger().warn(
					'Depth frame has almost no pixels within configured range; using fallback visualization. '
					f'valid_ratio={valid_ratio:.2%} depth_range=[{self.depth_min:.2f},{self.depth_max:.2f}] m'
				)
				self._last_warn = now

		# Use n for edge/contour detection
		grad_x = cv2.Sobel(n, cv2.CV_64F, 1, 0, ksize=3)
		grad_y = cv2.Sobel(n, cv2.CV_64F, 0, 1, ksize=3)
		magnitude = cv2.magnitude(grad_x, grad_y)
		MAX_VALUE = 255
		if np.max(magnitude) > 0:
			magnitude = cv2.normalize(magnitude, None, 0, MAX_VALUE, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
		else:
			magnitude = np.zeros_like(magnitude, dtype=np.uint8)

		ret, binary_mask = cv2.threshold(magnitude, self.threshold.data, MAX_VALUE, cv2.THRESH_BINARY)
		self._threshold_publisher.publish(self.threshold)

		contornos, hierarquia = cv2.findContours(
			binary_mask,
			cv2.RETR_TREE,
			cv2.CHAIN_APPROX_SIMPLE
		)

		output_contornos = np.zeros((binary_mask.shape[0], binary_mask.shape[1], 3), dtype=np.uint8)

		centroid_closest_to_center = None
		squared_dist_to_centroid = None
		cx = None
		cy = None
		size_squared = lambda x, y: x*x + y*y

		for i, contorno in enumerate(contornos):
			area = cv2.contourArea(contorno)
			cx_cont = None
			cy_cont = None
			_, _, w_box, h_box = cv2.boundingRect(contorno)
			if area > 100 and w_box > 50 and h_box > 50:
				cv2.drawContours(output_contornos, contornos, i, (0, 255, 0), 2)
				M = cv2.moments(contorno)
				if M["m00"] != 0:
					cx_cont = int(M["m10"] / M["m00"]) - center_x
					cy_cont = int(M["m01"] / M["m00"]) - center_y
				casco_convexo = cv2.convexHull(contorno)
				area_casco = cv2.contourArea(casco_convexo)
				if area_casco > 0:
					solidez = float(area)/area_casco
					if solidez > 0.8 and cx_cont is not None and cy_cont is not None:
						dist = size_squared(cx_cont, cy_cont)
						if not centroid_closest_to_center or dist < squared_dist_to_centroid:
							centroid_closest_to_center = (cx_cont, cy_cont)
							squared_dist_to_centroid = dist
						cv2.circle(output_contornos, (cx_cont+center_x, cy_cont+center_y), 5, (0, 0, 255), -1)

		if centroid_closest_to_center is not None:
			cx, cy = centroid_closest_to_center

		if cx is not None and cy is not None:
			cx_normalized = float(cx) / (w / 2.0)
			cy_normalized = float(cy) / (h / 2.0)
			centroid = Point()
			centroid.x = -1.0 * cy_normalized
			centroid.y = cx_normalized
			centroid.z = 0.0
			self._centroid_publisher.publish(centroid)
			self._window_found_publisher.publish(Bool(data=True))
			self.get_logger().info(f'Centroid coordinates (normalized): x={centroid.x:.3f}, y={centroid.y:.3f}')
			self.get_logger().debug(f'Raw pixel coordinates: cx={cx}, cy={cy}')
		else:
			self._window_found_publisher.publish(Bool(data=False))
			self.get_logger().info("No centroid found")

		now = time.time()
		if self._debug_period == 0.0 or (now - self._last_debug_pub_ts) >= self._debug_period:
			try:
				mask_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')
				ann_msg = self.bridge.cv2_to_imgmsg(output_contornos, encoding='bgr8')
				self._mask_pub.publish(mask_msg)
				self._annotated_pub.publish(ann_msg)
			except CvBridgeError as e:
				self.get_logger().error(f'CvBridge Error (publish debug): {e}')
			self._last_debug_pub_ts = now
		cv2.waitKey(1)


def main(args=None):
	rclpy.init(args=args)
	wf = WindowFinder()
	rclpy.spin(wf)
	wf.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()