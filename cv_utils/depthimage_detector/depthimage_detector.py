import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class WindowFinder(Node):
	def __init__(self):
		super().__init__('window_finder')

		self._centroid_publisher = self.create_publisher(Point, 'centroid', 10)
		self._threshold_publisher = self.create_publisher(UInt8, 'threshold', 10)
		self._subscriber = self.create_subscription(
			CompressedImage,
			'/depth_camera/compressedDepth',
			self.depth_image_callback,
			10
		)
		self.bridge = CvBridge()

		self.threshold: UInt8 = UInt8()
		self.threshold.data = int(8) # valor do threshold
	
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

		(height, width) = depth_meters.shape[:2]
		center_x = width // 2
		center_y = height // 2

		grad_x = cv2.Sobel(depth_meters, cv2.CV_64F, 1, 0, ksize=3)
		grad_y = cv2.Sobel(depth_meters, cv2.CV_64F, 0, 1, ksize=3)

		# aplicando o threshold:
		# pixel > t => pixel fica maximo
		# pixel < t => pixel fica minimo
		magnitude = cv2.magnitude(grad_x, grad_y)

		# normalizando para o intervalo 0-255
		MAX_VALUE = 255
		if np.max(magnitude) > 0:
			magnitude = cv2.normalize(magnitude, None, 0, MAX_VALUE, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
		else:
			magnitude = np.zeros_like(magnitude, dtype=np.uint8)

		ret, binary_mask = cv2.threshold(magnitude, self.threshold.data, MAX_VALUE, cv2.THRESH_BINARY)
		self._threshold_publisher.publish(self.threshold)

		#cv2.imshow('Binary Mask', binary_mask)
		#cv2.waitKey(1)

		contornos, hierarquia = cv2.findContours(
			binary_mask,
			cv2.RETR_TREE, # hierarquia completa
			cv2.CHAIN_APPROX_SIMPLE # modo de aproximação simples
		)

		output_contornos = np.zeros((binary_mask.shape[0], binary_mask.shape[1], 3), dtype=np.uint8)

		centroid_closest_to_center = None
		squared_dist_to_centroid = None
		size_squared = lambda x,y: x*x + y*y

		for i, contorno in enumerate(contornos):
			area = cv2.contourArea(contorno)
			
			# coordenadas do centroid de cada iteracao
			cx = None
			cy = None
			
			# obtendo os tamanhos dos contornos
			_, _, w, h = cv2.boundingRect(contorno)

			if area > 100 and w > 50 and h > 50:
				cv2.drawContours(output_contornos, contornos, i, (0, 255, 0), 2)
				
				'''
				Técnica dos Momentos de Imagem (para encontrar o centróide de uma imagem)
				
				"momentos" são uma forma de medir distribuição de massa. A partir disso, podemos conseguir área, orientação, centroide, ...
				eles são, basicamente, uma média ponderada entre as intensidades dos pixels.

				c_x = soma_de_todas_as_coordenadas_x_dos_pixels_da_forma / área_do_contorno
				c_y = soma_de_todas_as_coordenadas_y_dos_pixels_da_forma / área_do_contorno
				'''


				M = cv2.moments(contorno)
				if M["m00"] != 0:
					cx = int(M["m10"] / M["m00"]) - center_x
					cy = int(M["m01"] / M["m00"]) - center_y
				
				'''
				OBSERVAÇÃO:
				para o cenário em que não é possível ver a janela completa e que, na verdade, ela fique em um formato, digamos,
				como um "L", seu centróide não pertence ao interior do contorno!!
				'''

				'''
				MÉTODO DA ANÁLISE DE SOLIDEZ

				A solidez mede o quão "convexo" é um objeto.
				Ela é calculada como a razão entre a área do contorno e a área do seu "casco convexo" (convex hull)
				Mas o que seria esse "casco convexo" ?
				Pense nele como esticar um elástico ao redor de todos os pontos do seu contorno
				'''

				casco_convexo = cv2.convexHull(contorno)
				area_casco = cv2.contourArea(casco_convexo)

				if area_casco > 0:
					solidez = float(area)/area_casco
					if solidez > 0.8:
						dist = size_squared(cx, cy)
						if not centroid_closest_to_center or dist < squared_dist_to_centroid:
							centroid_closest_to_center = (cx, cy)
							squared_dist_to_centroid = dist

						cv2.circle(output_contornos, (cx+center_x, cy+center_y), 5, (0, 0, 255), -1) # temos que somar novamente para voltar ao sistema de coordenadas padrão

		if cx and cy:
			centroid = Point()
			centroid.x = -1*float(cy)
			centroid.y = float(cx)
			centroid.z = 0.0
			self._centroid_publisher.publish(centroid)
			self.get_logger().info(f'Centroid coordinates: x={centroid.x}, y={centroid.y}')
		else:
			self.get_logger().info("No centroid found")

		cv2.imshow('Contours Found', output_contornos)
		cv2.waitKey(1)


def main(args=None):
	rclpy.init(args=args)
	wf = WindowFinder()
	rclpy.spin(wf)
	wf.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()