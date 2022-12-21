import time
import math


class Odometry:

	def __init__(self):
		#Constructor initializations --- 
		self.prev_lencoder = 0
		self.prev_rencoder = 0
		self.port = "/dev/ttyACM0"
		self.enc_l = 0 
		self.enc_r = 0
		self.lmult = 0
		self.rmult = 0
		self.left = 0
		self.right = 0
		self.encoder_min = -2147483648
		self.encoder_max = 2147483648
		self.ticks_meter = 75021
		self.base_width = 0.475
		self.encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min
		self.encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min
		self.enc_left = 0
		self.enc_right = 0

		self.x_final = 0
		self.y_final = 0
		self.theta_final = 0
		# ---------- Write code to connect to the controller.----------------
		# device.Connect(port);                  
		# int conn_status = device.Connect(port);
		# if(conn_status != RQ_SUCCESS){
		#     print("Error Connecting to Roboteq device: " + conn_status)
		#     print("Please Check Following Conditions!!")
		#     print("       Check Serial Cable.")
		#     print("		Check MCB status.")
		#     print("		Check Toggle Switch.")
		# }


	def get_wheel_enc(self):
		# ---------- Gets the wheel encoder values for calculations --------
		time.sleep(0.01)   # -- Sleep for 10ms ---
		## Get the left encoder values and wrap it ------
		if(device.GetValue(_CSS,1,self.enc_l) == RQ_SUCCESS):  # ----- Write this in python API way ------
			if((self.enc_l < self.encoder_low_wrap) and (self.prev_lencoder > self.encoder_high_wrap)):
				self.lmult = self.lmult + 1

			if((self.enc_l > self.encoder_high_wrap) and (self.prev_lencoder < self.encoder_low_wrap)):
				self.lmult = self.lmult - 1 

			self.left = 1.0 * (self.enc_l + self.lmult * (self.encoder_max - self.encoder_min ))
			self.prev_lencoder = self.enc_l;
		else:
			print("Error getting left encoder values!!")

		time.sleep(0.01)

	    ## Get the right encoder values and wrap it ------

		if(device.GetValue(_CSS,2,self.enc_r) == RQ_SUCCESS):  # ----- Write this in python API way ------
			if((self.enc_r < self.encoder_low_wrap) and (self.prev_rencoder > self.encoder_high_wrap)):
				self.rmult = self.rmult + 1

			if((self.enc_r > self.encoder_high_wrap) and (self.prev_rencoder < self.encoder_low_wrap)):
				self.rmult = self.rmult - 1 

			self.right = 1.0 * (self.enc_r + self.rmult * (self.encoder_max - self.encoder_min ))
			self.prev_rencoder = self.enc_r;
		else:
			print("Error getting right encoder values!!")



	def dist_corr(self,distance,direc):
		dist_l, dist_r, dist_c, th, x, y = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
		throttle = direc*200 # ASSIGN THROTTLE VALUE
		steering = 0       # ASSIGN STEERING VALUE
		self.x_final, self.y_final, self.theta_final = 0.0, 0.0, 0.0

		x_prev, y_prev, path_len = 0.0, 0.0, 0.0

		while True:
			self.motor_comand(throttle, steering) #------ Add the motor command method -------
			self.get_wheel_enc()

			if(self.enc_left == 0):
				dist_l, dist_r = 0.0

			else:
				dist_l = (self.left - self.enc_left)/(self.ticks_meter)
				dist_r = (self.right - self.enc_right)/(self.ticks_meter)

			self.enc_left = self.left
			self.enc_right = self.right
			self.dist_c = (self.dist_l + self.dist_r)/2.0
			th = (dist_r - dist_l)/self.base_width

			if(dist_c != 0):
				x = math.cos(th)*dist_c
				y = -1*math.sin(th)*dist_c
				self.x_final = self.x_final + ( math.cos( self.theta_final ) * x - math.sin( self.theta_final ) * y )
				self.y_final = self.y_final + ( math.sin( self.theta_final ) * x + math.cos( self.theta_final ) * y )
			if(th != 0):
				self.theta_final = self.theta_final + th

			path_len = math.sqrt((self.x_final-x_prev)**2 + (self.y_final-y_prev)**2) + path_len
			x_prev = self.x_final
			y_prev = self.y_final

			print("X : " + self.x_final)
			print("Y : " + self.y_final)

			if(abs(self.x_final) >= distance):
				break

		self.motor_command(0,0) #----- Give motor command ----


	def ange_corr(self, angle, direc):
		dist_l, dist_r, dist_c, th, x, y = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
		throttle = 0 # ASSIGN THROTTLE VALUE
		steering = direc*30       # ASSIGN STEERING VALUE
		self.x_final, self.y_final, self.theta_final = 0.0, 0.0, 0.0

		x_prev, y_prev, path_len = 0.0, 0.0, 0.0

		while True:
			self.motor_comand(throttle, steering) #------ Add the motor command method -------
			self.get_wheel_enc()

			if(self.enc_left == 0):
				dist_l, dist_r = 0.0

			else:
				dist_l = (self.left - self.enc_left)/(self.ticks_meter)
				dist_r = (self.right - self.enc_right)/(self.ticks_meter)

			self.enc_left = self.left
			self.enc_right = self.right
			self.dist_c = (self.dist_l + self.dist_r)/2.0
			th = (dist_r - dist_l)/self.base_width

			if(dist_c != 0):
				x = math.cos(th)*dist_c
				y = -1*math.sin(th)*dist_c
				self.x_final = self.x_final + ( math.cos( self.theta_final ) * x - math.sin( self.theta_final ) * y )
				self.y_final = self.y_final + ( math.sin( self.theta_final ) * x + math.cos( self.theta_final ) * y )
			if(th != 0):
				self.theta_final = self.theta_final + th

			print("Angle : " + theta_final)	
			if(abs(self.theta_final*180/math.pi) >= angle*0.98):
				break

		self.motor_command(0,0) #----- Give motor command ----

	def motor_command(self,thr, stree):
		conn_status = 0
		if((conn_status = device.SetCommand(_GO,1,thr)) != RQ_SUCCESS): ## Write Python method
			print("Failed in Motor Throttle -- > " + conn_status)
		if((conn_status = device.SetCommand(_GO,2,stree)) != RQ_SUCCESS): ## Write Python method
			print("Failed in Motor Steering -- > " + conn_status)
		time.sleep(0.01)




