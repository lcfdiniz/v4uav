#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import *
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import thread
import time
import numpy as np
import pandas as pd
from follower.msg import Follower as FollowerMsg
from follower.srv import *

# Função auxiliar para avaliar proximidade entre valores
def is_near(msg, x, y, valor):
	return abs(x - y) < valor

# Requisição do serviço de detecção de linhas de transmissão
def detection_request(mode):
	rospy.wait_for_service('detection')
	try:
		detection = rospy.ServiceProxy('detection', Detection)
		resp = detection(mode)
		return resp
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

# Ajusta o valor do ângulo de guinada (_yaw) entre -180 e 180 graus
def ajusta_yaw(_yaw):
	if abs(_yaw) > np.pi:
		if _yaw > 0:
			_yaw = _yaw - 2*np.pi
		if _yaw < 0:
			_yaw = 2*np.pi + _yaw
	
	return _yaw

# Classe que armazena a posição inercial (x,y,z) e ângulo de guinada (yaw) do drone
class DronePosition:
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.yaw = 0.0
		self.df = pd.DataFrame(columns=['t', 'x', 'y', 'z'])

# Classe para transição entre modos de voo
class fcuModes:
	def setTakeoff(self):
		rospy.wait_for_service('mavros/cmd/takeoff')
		try:
			takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
			takeoffService(altitude = 3)
		except rospy.ServiceException, e:
			print("Service takeoff call failed: %s"%e)

	def setArm(self, _to_arm):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(_to_arm)
		except rospy.ServiceException, e:
			print("Service takeoff call failed: %s"%e)

	def setMode(self, _mode):
		if _mode in ("STABILIZED",
					 "OFFBOARD",
					 "ALTCTL",
					 "POSCTL",
					 "AUTO.LAND",
					 "AUTO.RTL"):
			rospy.wait_for_service('mavros/set_mode')
			try:
				flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
				while not state.mode == _mode:
					flightModeService(custom_mode = _mode)
					rate.sleep()
			except rospy.ServiceException, e:
				print("service set_mode call failed: %s. Offboard Mode could not be set."%e)

# Classe para geração de setpoints de posição inercial e ângulo de guinada
class SetPosition:
	def init(self, _x, _y, _z, _yaw, _takeoff=False):
		self.x = _x
		self.y = _y
		self.z = _z
		self.yaw = ajusta_yaw(_yaw)
		self.takeoff = _takeoff

		self.done_evt = threading.Event()

		# Publisher para mavros/setpoint_raw/local
		self.pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
		# Subscriber para mavros/local_position/pose
		self.sub = rospy.Subscriber("/mavros/local_position/pose", SP.PoseStamped, self.reached)	

	def start(self):
		self.activated = True

		try:
			thread.start_new_thread(self.navigate, ())
		except:
			fault("Erro: Incapaz de iniciar a thread")

	def finish(self):
		rospy.loginfo("Encerrando o módulo SetPosition")
		self.sub.unregister()
		self.activated = False
    
	def navigate(self):
		msg = PositionTarget(
			header=SP.Header(
				frame_id="world",
				stamp=rospy.Time.now()),
			coordinate_frame=1
		)

		while not rospy.is_shutdown():
			if not self.activated:
				break

			# Preenchendo mensagem
			msg.position.x = self.x
			msg.position.y = self.y
			msg.position.z = self.z
			msg.yaw = self.yaw

			# Publica a mensagem
			self.pub.publish(msg)
			rate.sleep()

	def set(self, _x, _y, _z, _yaw, delay=0, wait=False):
		self.done_evt.clear()
		self.x = _x
		self.y = _y
		self.z = _z
		self.yaw = ajusta_yaw(_yaw)

		if wait:
			while not self.done_evt.is_set() and not rospy.is_shutdown():
				rate.sleep()

		if delay > 0:
			time.sleep(delay)

	def reached(self, topic):
		quat = topic.pose.orientation
		euler_angles = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		# Tratando descontinuidades de ângulos entre -180 e +180
		if abs(self.yaw) > (np.pi - 0.1) and abs(euler_angles[2]) > (np.pi - 0.1):
			topic_yaw = (self.yaw/abs(self.yaw))*abs(euler_angles[2])
		else:
			topic_yaw = euler_angles[2]

		if is_near('X', topic.pose.position.x, self.x, 0.5) and \
			is_near('Y', topic.pose.position.y, self.y, 0.5) and \
			is_near('Z', topic.pose.position.z, self.z, 0.5) and \
			is_near('Yaw', topic_yaw, self.yaw, 0.1):
			
			self.done_evt.set()

		drone_pos.x = topic.pose.position.x
		drone_pos.y = topic.pose.position.y
		drone_pos.z = topic.pose.position.z
		drone_pos.yaw = euler_angles[2]
		entrada = {'t': time.time(),
				   'x': drone_pos.x,
				   'y': drone_pos.y,
				   'z': drone_pos.z}
		drone_pos.df = drone_pos.df.append(entrada, ignore_index=True)

# Classe para geração de setpoints
class FollowLines:
	def init(self, _vx, _z, _yaw, _dy=0.0):
		self.vx = _vx
		self.z = _z
		self.yaw = ajusta_yaw(_yaw)
		self.dy = _dy
		
		# Task 0: Iddle; Task 1: Follow; Task 2: Land
		self.task = 0
		self.task_2_stage = 0
		
		self.tempo_max = 5.0
		self.tempo_min = 2.0
		self.clock = time.time()

		# Publisher para mavros/setpoint_raw/local
		self.pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
		# Subscriber para mavros/local_position/pose
		self.sub = rospy.Subscriber("/mavros/local_position/pose", SP.PoseStamped, self.reached)

	def start(self):
		self.activated = True

		try:
			thread.start_new_thread(self.navigate, ())
		except:
			fault("Erro: Incapaz de iniciar a thread")

	def finish(self):
		rospy.loginfo("Encerrando o módulo FollowLines")
		self.sub.unregister()
		self.activated = False
    
	def navigate(self):
		msg = PositionTarget(
			header=SP.Header(
				frame_id="body",
				stamp=rospy.Time.now()),
			type_mask=0b0000101111000111,
			coordinate_frame=8
		)

		while not rospy.is_shutdown():
			if not self.activated:
				break
			
			# Controla a velocidade em y no referencial do corpo
			KPy = 0.1
			vy = KPy * self.dy

			# Controlador P para a altitude
			KPz = 0.5
			erro_z = self.z - drone_pos.z
			vz = KPz * erro_z

			# Limite para a velocidade vertical máxima
			max_up_vel = 5.0
			vz = min(vz, max_up_vel)

			msg.velocity.x = self.vx
			msg.velocity.y = vy
			msg.velocity.z = vz
			msg.yaw = self.yaw - np.pi/2 # Após reinstalação do ROS, foi necessário defasar em -90 graus

			# Publica a mensagem
			self.pub.publish(msg)
			rate.sleep()

	def handle_task(self, tempo):
		
		# Task Hold (utilizada apenas na inicialização do módulo)
		if self.task == 0:
			self.vx = 0.0
			self.z = drone_pos.z
			self.yaw = drone_pos.yaw
			self.dy = 0.0

			self.tempo_max = 5.0
			self.tempo_min = 2.0
			self.clock = time.time()

		# Task Line Follower
		if self.task == 1:
			
			self.tempo_max = 5.0
			# Requisita o serviço de detecção
			resp = detection_request(mode=1)
			if resp.ld > 0:
				# Cálculo da ângulação relativa
				yaw_sp =  ajusta_yaw(drone_pos.yaw + radians(resp.th[0]))
				self.vx = 2.5
				self.yaw = yaw_sp
				self.tempo_min = max(abs(resp.th[0])/22.5,0.5)			

				# Cálculo da altura relativa
				h_alvo = 35.0 # Define a altura relativa alvo
				h_tol = 0.5 # Define a tolerância admitida para a altura relativa
				delta_h = h_alvo - max(resp.rh)
				if abs(delta_h) > h_tol and max(resp.rh) > 0.0:
					z_sp = drone_pos.z + delta_h
					self.z = z_sp
					
				# Cálculo do deslocamento lateral
				y_tol = 0.5 # Define a tolerância admitida para o deslocamento lateral
				delta_y = max(resp.dy)
				if abs(delta_y) > y_tol:
					self.dy = delta_y
			
				self.clock = time.time()

			if tempo > 10.0:
				rospy.loginfo("Linha de transmissão não detectada. Retornando para o modo Hold")
				data = FollowerMsg(x=drone_pos.x,
								   y=drone_pos.y,
								   z=drone_pos.z,
								   yaw=drone_pos.yaw,
								   mode=3)
				self.task = 0
				cmdCallback(data)
			
		# Task Land In Line
		if self.task == 2:
			
			# Estágio 0: Posiciona o veículo a 15 metros da linha de transmissão
			if self.task_2_stage == 0:
				# Para o veículo
				self.vx = 0.0
				self.z = drone_pos.z
				self.dy = 0.0
				self.tempo_max = 60.0
				
				# Requisita o serviço de detecção
				resp = detection_request(mode=1)
				
				# Apenas um segmento de linha detectado
				if resp.ld == 1:
					# Cálculo da ângulação relativa
					yaw_sp =  ajusta_yaw(drone_pos.yaw + radians(resp.th[0]))
					self.yaw = yaw_sp
					self.tempo_min = max(abs(resp.th[0])/22.5,0.5)			

					# Cálculo da altura relativa
					h_alvo = 15.0 # Define a altura relativa alvo
					h_tol = 0.25 # Define a tolerância admitida para a altura relativa
					delta_h = h_alvo - max(resp.rh)
					if abs(delta_h) > h_tol and max(resp.rh) > 0.0:
						z_sp = drone_pos.z + delta_h
						self.z = z_sp
					
					# Cálculo do deslocamento lateral
					y_tol = 0.5 # Define a tolerância admitida para o deslocamento lateral
					delta_y = max(resp.dy)
					if abs(delta_y) > y_tol:
						self.dy = delta_y
			
					# Caso as condições sejam atendidas, avança para o próximo estágio
					if resp.th[0] < 5.0 and delta_h < h_tol and delta_y < y_tol:
						self.task_2_stage = 1

					self.clock = time.time()
				
				# Mais de um segmento de linha detectado
				elif resp.ld > 1:
					rospy.loginfo("Mais de um segmento de linha detectado. Retornando para o modo Hold")
					data = FollowerMsg(x=drone_pos.x,
									   y=drone_pos.y,
									   z=drone_pos.z,
									   yaw=drone_pos.yaw,
									   mode=3)
					self.task = 0
					cmdCallback(data)

				# Mais de 90 segundos sem detectar a linha
				if tempo > 90.0:
					rospy.loginfo("Linha de transmissão não detectada. Retornando para o modo Hold")
					data = FollowerMsg(x=drone_pos.x,
									   y=drone_pos.y,
									   z=drone_pos.z,
									   yaw=drone_pos.yaw,
									   mode=3)
					self.task = 0
					cmdCallback(data)
			
			# Estágio 1: Centraliza o drone com a linha e desce até a linha central
			elif self.task_2_stage == 1:
				# Para o veículo
				self.vx = 0.0
				self.z = drone_pos.z
				self.dy = 0.0
				self.tempo_max = 60.0
				self.tempo_min = 4.0

				# Requisita o serviço de detecção
				resp = detection_request(mode=1)
				
				# Cálculo do deslocamento lateral
				y_tol = 0.15 # Define a tolerância admitida para o deslocamento lateral
				delta_y = max(resp.dy)
				if abs(delta_y) < y_tol:
					resp = detection_request(mode=2)
					self.z = self.z - max(resp.rh) - 1.3
					self.task_2_stage = 2
				else:
					self.dy = 2.5*delta_y # Para corrigir em 4s

				self.clock = time.time()

			# Estágio 2: Centraliza o drone com a linha central e pousa sobre a mesma
			elif self.task_2_stage == 2:
				self.tempo_max = 10.0
				self.tempo_min = 4.0
				
				# Requisita o serviço de detecção
				resp = detection_request(mode=3)
				if abs(resp.dy[0]) < 0.05:
					self.z = self.z - 5.0
					self.task_2_stage = 3
				else:
					self.dy = 2.5*resp.dy[0] # Para corrigir em 4s

				self.clock = time.time()

			# Estágio 3: Drone pousado com sucesso
			elif self.task_2_stage == 3:
				rospy.loginfo("Pousado com sucesso! Retornando para o modo Hold")
				print("X: " + str(round(drone_pos.x,2)))
				print("Y: " + str(round(drone_pos.y,2)))
				print("Z: " + str(round(drone_pos.z,2)))
				print("Yaw: " + str(round(drone_pos.yaw,2)))
				data = FollowerMsg(x=drone_pos.x,
								   y=drone_pos.y,
								   z=drone_pos.z,
								   yaw=drone_pos.yaw,
								   mode=3)
				self.task = 0
				self.task_2_stage = 0
				cmdCallback(data)

	def reached(self, topic):
		quat = topic.pose.orientation
		euler_angles = euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
		# Tratando descontinuidades de ângulos entre -180 e +180
		if abs(self.yaw) > (np.pi - 0.1) and abs(euler_angles[2]) > (np.pi - 0.1):
			topic_yaw = (self.yaw/abs(self.yaw))*abs(euler_angles[2])
		else:
			topic_yaw = euler_angles[2]
		# Tempo decorrido após a última detecção positiva
		tempo = time.time() - self.clock

		if is_near('Z', topic.pose.position.z, self.z, 0.5) and \
			is_near('Yaw', topic_yaw, self.yaw, 0.1) and \
			tempo > self.tempo_min or \
			tempo > self.tempo_max:
			
			self.handle_task(tempo)

		drone_pos.x = topic.pose.position.x
		drone_pos.y = topic.pose.position.y
		drone_pos.z = topic.pose.position.z
		drone_pos.yaw = euler_angles[2]
		entrada = {'t': time.time(),
				   'x': drone_pos.x,
				   'y': drone_pos.y,
				   'z': drone_pos.z}
		drone_pos.df = drone_pos.df.append(entrada, ignore_index=True)

state = State()
def stateCb1(msg):
	global state
	state = msg

extend_state = ExtendedState()
def stateCb2(msg):
	global extend_state
	extend_state = msg

def cmdCallback(data):
	x = data.x
	y = data.y
	z = data.z
	yaw = data.yaw
	mode = data.mode

	# Modo Takeoff
	if mode == 0:
		if set_pos.takeoff:
			rospy.loginfo("Modo Takeoff")	
			while not state.armed:
				modes.setArm(True)
				rate.sleep()
			rospy.loginfo("Drone armado")
			set_pos.set(0.0, 0.0, z, 0.0, wait=True)
			set_pos.takeoff = False
			rospy.loginfo("Takeoff realizado")
		else:
			rospy.loginfo("Takeoff já realizado")
	
	else:
		if not set_pos.takeoff:

			# Modo Set Position
			if mode == 1:
				rospy.loginfo("Modo Set Position")
				if set_pos.activated:
					set_pos.set(x, y, z, yaw, wait=True)
					rospy.loginfo("Set Position realizado")
				elif follow_lines.activated:
					set_pos.init(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					set_pos.start()
					follow_lines.finish()
					set_pos.set(x, y, z, yaw, wait=True)
					rospy.loginfo("Set Position realizado")
				else:
					set_pos.init(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					set_pos.start()
					set_pos.set(x, y, z, yaw, wait=True)
					rospy.loginfo("Set Position realizado")

			# Modo Set Relative Position
			elif mode == 2:
				rospy.loginfo("Modo Set Relative Position")
				if set_pos.activated:
					set_pos.set(drone_pos.x + x, drone_pos.y + y, drone_pos.z + z, drone_pos.yaw + yaw, wait=True)
					rospy.loginfo("Set Relative Position realizado")
				elif follow_lines.activated:
					set_pos.init(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					set_pos.start()
					follow_lines.finish()
					set_pos.set(drone_pos.x + x, drone_pos.y + y, drone_pos.z + z, drone_pos.yaw + yaw, wait=True)
					rospy.loginfo("Set Relative Position realizado")
				else:
					set_pos.init(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					set_pos.start()
					set_pos.set(drone_pos.x + x, drone_pos.y + y, drone_pos.z + z, drone_pos.yaw + yaw, wait=True)
					rospy.loginfo("Set Relative Position realizado")

			# Modo Hold
			elif mode == 3:
				rospy.loginfo("Modo Hold")
				if set_pos.activated:
					set_pos.set(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					rospy.loginfo("Hold realizado")
				elif follow_lines.activated:
					set_pos.init(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					set_pos.start()
					follow_lines.finish()
					rospy.loginfo("Hold realizado")
				else:
					set_pos.init(drone_pos.x, drone_pos.y, drone_pos.z, drone_pos.yaw)
					set_pos.start()
					rospy.loginfo("Hold realizado")
			
			# Modo Get Position
			elif mode == 4:
				rospy.loginfo("Modo Get Position")
				print("X: " + str(round(drone_pos.x,2)))
				print("Y: " + str(round(drone_pos.y,2)))
				print("Z: " + str(round(drone_pos.z,2)))
				print("Yaw: " + str(round(drone_pos.yaw,2)))
			
			# Modo Follow Lines
			elif mode == 5:
				rospy.loginfo("Modo Follow Lines")
				if set_pos.activated:
					follow_lines.init(0.0, drone_pos.z, drone_pos.yaw)
					follow_lines.start()
					set_pos.finish()
					follow_lines.task = 1
				elif follow_lines.activated:
					follow_lines.task = 1
				else:
					follow_lines.init(0.0, drone_pos.z, drone_pos.yaw)
					follow_lines.start()
					follow_lines.task = 1
			
			# Modo Land In Line
			elif mode == 6:
				rospy.loginfo("Modo Land In Line")
				if set_pos.activated:
					follow_lines.init(0.0, drone_pos.z, drone_pos.yaw)
					follow_lines.start()
					set_pos.finish()
					follow_lines.task = 2
				elif follow_lines.activated:
					follow_lines.task = 2
				else:
					follow_lines.init(0.0, drone_pos.z, drone_pos.yaw)
					follow_lines.start()
					follow_lines.task = 2
			
			# Modo Return To Land
			elif mode == 7:
				rospy.loginfo("Modo Return To Land")
				if set_pos.activated:
					set_pos.finish()
				try: 
					if follow_lines.activated:
						follow_lines.finish()
				except AttributeError:
					pass
				modes.setMode("AUTO.RTL")
				while state.armed:
					rate.sleep()
				rospy.signal_shutdown('detection_client')

			else:
				rospy.loginfo("Modo inválido")
		
		else:
			rospy.loginfo("Realize o Takeoff antes de proceder com a ação")

def detection_client():
	sub = rospy.Subscriber("/follower/commands", FollowerMsg, cmdCallback)
	rospy.loginfo("Aguardando comando de Takeoff")
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Encerrando o programa")

modes = fcuModes()
drone_pos = DronePosition()
set_pos = SetPosition()
follow_lines = FollowLines()

if __name__ == '__main__':
    try:
        rospy.init_node('detection_client')
        mavros.set_namespace()
        rate = rospy.Rate(20)

        state = State()
        rospy.Subscriber('mavros/state', State, stateCb1)
        extend_state = ExtendedState()
        rospy.Subscriber('mavros/extended_state', ExtendedState, stateCb2)
        
        rospy.loginfo("Iniciando módulo SetPosition")
        set_pos.init(0.0, 0.0, 0.0, 0.0, _takeoff=True)
        set_pos.start()

        rospy.loginfo("Iniciando controle OFFBOARD")
        modes.setMode("OFFBOARD")

        detection_client()

        drone_pos.df.to_csv("tracking.csv")
        rospy.loginfo("Encerrando o programa")

    except rospy.ROSInterruptException:
		drone_pos.df.to_csv("tracking.csv")
		pass
