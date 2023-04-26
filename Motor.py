from TMCL.motor import AxisParameterInterface
from serial import Serial
from time import sleep
import TMCL
#step -> angle = 1.8 °
class Stepper:
    def __init__(self,serial_name,address=0,max_speed=8000,p_limit=-10000000,n_limit=10000000):
        self.serial_name = serial_name
        self.MODULE_ADDRESS = address
        self.serial_port = Serial(self.serial_name) #il nome della porta dipende dal sistema operativo, per windows "COM3" per Linux forse ""/dev/tty.usbmodem1241" (controllare)
        ##Create a Bus instance using the open seri"al port
        self.bus = TMCL.connect(self.serial_port)
        ##Get the motor
        self.motor = self.bus.get_motor(self.MODULE_ADDRESS)
        #motor è l'oggetto che usiamo per i vari comandi, che ci permettono di usare lo pseudocodice TMCL, nel file motor si possono trovare i vari metodi nella classe Motor
        ##From this point you can start issuing TMCL commands
        ##to the motor as per the TMCL docs.
        self.setting = AxisParameterInterface(self.motor)
        self.setting.max_positioning_speed = max_speed
        self.max_speed=max_speed
        self.positive_limit=p_limit
        self.negative_limit=n_limit
        self.setting.max_current =255  #160  # more current less torque --> more sensitivity
        # questo parametro definisce la corrente usata dal motore fermo..
        #self.setting.standby_current = 8
        self.position=self.setting.actual_position #actual position set as zero
        #self.setting.actual_position = 0

    def move_stepper(self,step):
        desired_position = step+self.position
        #if desired_position < self.positive_limit and desired_position > self.negative_limit:
        print(f"turn: {desired_position}")
        self.setting.set(181, 0)
            #self.motor.move_relative(step)
        self.motor.move_absolute(desired_position)
        
        return
        while True:
            
            #endless loop per un controllo periodico se l'angolo di sterzo richiesto è raggiunto oppure il limite è superato
            if self.setting.target_position_reached==True :
                break #posizione raggiunta esci dal loop
            if  self.setting.actual_position>self.positive_limit:
                #ad alte velocità, se non vengono settati valori di decelarzione alti e/o di velocità di stop elevati, il motore compie un safe stop e per questo va oltre al valore di stop richiesto
                self.motor.move_absolute(self.positive_limit) #con questo il motore viene riportato comunque ad una posizione precisa
                #motor.stop() #con questo metodo il motore compie un safe stop senza un posizione finale precisa di stop
                print('stop,over positive limit')
                break #limite raggiunto fermati ed esci dal loop
            if  self.setting.actual_position<self.negative_limit:
                #ad alte velocità, se non vengono settati valori di decelarzione alti e/o di velocità di stop elevati, il motore compie un safe stop e per questo va oltre al valore di stop richiesto
                self.motor.move_absolute(self.negative_limit) #con questo il motore viene riportato comunque ad una posizione precisa
                #motor.stop() #con questo metodo il motore compie un safe stop senza un posizione finale precisa di stop
                print('stop,over negative limit')
                break #limite raggiunto fermati ed esci dal loop

            sleep(0.001)
            self.position = self.setting.actual_position
            print('position:', self.position)
        self.position = self.setting.actual_position #salvo la posizione del motore attuale
   


    def brake(self):

        max_velocity = 2500  # less velocity more torque --> less sensitivity to reach SG=0

        #sgvel = max_velocity-100
        self.setting.standby_current = 100
        self.setting.max_positioning_speed = max_velocity
        self.setting.max_accelleration = 1500 # parameter number 5
        # parameters 12 and 13 will disable stop on right and left switches respectively (0 active, 1 inactive)
        self.setting.set(16, 0) # will disable initial acc phase if set to 0, sets intital acc target velocity otherwise
        #setting.set(18,7629278) # deceleration value before reaching sto
        self.setting.max_current = 255

        self.setting.set(181, 0)
        self.motor.move_relative(-65000)

        sleep(10)
        self.motor.move_relative(-32000)
        self.setting.max_positioning_speed = 75
        self.setting.max_accelleration = 10
        return


    def release_brake(self):
        self.setting.max_positioning_speed = 2500
        self.setting.max_accelleration = 3000
        self.setting.move_relative(-self.setting.actual_position)
        return


    def calibrate(self):
        max_velocity = 4000  # less velocity more torque --> less sensitivity to reach SG=0
        sgvel = max_velocity-100
        self.setting.max_positioning_speed = max_velocity
        # setting.max_acceleration = 1000
        self.setting.max_current =255  #160  # more current less torque --> more sensitivity
        # questo parametro definisce la corrente usata dal motore fermo..
        self.setting.standby_current = 8
        # ..Deve essere il più basso possibile per evitare surriscaldamento...
        # ..Incide sullo Stallguard (più è alto più la sensibilità decresce).
        stallGuard_value = 63  # da definire accuratamente in fase di test
        self.setting.set(174, stallGuard_value)
        self.setting.set(181, 0)  # start with this value set to zero to let the motor start


        # Start calibration
        print('start')
        self.motor.rotate_right(max_velocity)
        sleep(1)
        while True:
            # here we set this value to activate the stall detection
            self.setting.set(181, sgvel)
            print('Actual stallguard:', self.setting.get(206))
            if self.setting.get(206) < 100:  # wait for stallguard egual to zero for stall detection
                print('Actual position:', self.setting.actual_position)
                positive_limit = self.setting.actual_position
                break

        self.setting.set(181, 0)  # Here to let the motor start again
        self.motor.rotate_left(max_velocity)
        sleep(2)

        while True:
            self.setting.set(181, sgvel)
            # print('Actual stallguard2:', setting.get(206))
            if self.setting.get(206) < 100:  # wait for stallguard egual to zero for stall detection
                print('Actual pos2:', self.setting.actual_position)
                negative_limit = self.setting.actual_position
                break

        zero = round((positive_limit+negative_limit)/2)

        range = positive_limit - negative_limit
        range_standard_h = 1000000
        range_standard_l = 100
        if range < range_standard_l | range > range_standard_h:
            sys.exit('Error in calibration')

        self.setting.set(181, 0)
        print('positive limit:', positive_limit)
        print('negative limit:', negative_limit)
        print('zero coordinate value:', zero)

        self.motor.move_absolute(zero)
        sleep(1)  
        self.setting.max_positioning_speed = self.max_speed
        return positive_limit,negative_limit