import cv2
import board
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import datetime
import numpy as np
import multiprocessing

TRIG=[23, 25, 7]
ECHO=[24, 8, 1]
DISTANCE=multiprocessing.Array('f', 3)
previous_time = datetime.datetime.now()
sensor_index = 0
window_name = "Sistem asistenta marsarier"

def init_gpio():
    # folosire numerotare pini definita in procesorul Broadcom
    GPIO.setmode(GPIO.BCM)
    
    # initializare pinii pentru fiecare senzor ultrasonic
    for i in range(3):
        GPIO.setup(TRIG[i], GPIO.OUT)
        GPIO.setup(ECHO[i], GPIO.IN)
        GPIO.output(TRIG[i], False)
        time.sleep(0.2)

def read_distances_from_sensor(index):
    global DISTANCE
    global TRIG
    global ECHO
    
    # emite un puls ultrasonic
    GPIO.output(TRIG[index], True)
    time.sleep(0.00001)
    GPIO.output(TRIG[index], False)

    # masurare timp pana la intoarcerea ecoului
    while GPIO.input(ECHO[index]) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO[index]) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end-pulse_start
    
    # calculare distanta din timp
    distance = pulse_duration * 17150
    
    # notare distanta in lista globala de distante
    DISTANCE[index] = distance


def draw_distances(frame):
    global DISTANCE
    
    # noteaza distantele pe ecran in culori in functie de marimea acestora
    text_positions = [[440, 460], [280, 460], [120, 460]]
    for i in range(0, 3):
        if DISTANCE[i] > 200:
            # daca distanta mai mare de 200 cm culoarea verde
            color = (0, 255, 0)
        elif DISTANCE[i] > 100:
            # daca distanta mai mare de 100 cm culoarea galben
            color = (0, 255, 255)
        elif DISTANCE[i] > 50:
            # daca distanta mai mare de 50 cm culoarea portocaliu
            color = (0, 130, 255)
        else:
            #altfel culoarea rosu
            color = (0, 0, 255)
        # afiseaza distanta pe ecran
        
        # rotunjeste distanta
        rounded_distance = round(DISTANCE[i], 2)
        
        # afiseaza distanta pe ecran
        frame = cv2.putText(frame, f'{rounded_distance} cm', text_positions[i], cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
 

def attempt_distance_measurement():
    global previous_time
    global sensor_index
    
    # calculare timp scurs de la ultima masuratoare de distanta
    current_time = datetime.datetime.now()
    elapsed_time = current_time-previous_time
    # daca a trecut suficient timp pentru a evita interferentele masuram
    if elapsed_time.total_seconds() > 0.2:
        # porneste un proces paralel pentru citirea distantei de la senzor
        read_process = multiprocessing.Process(target=read_distances_from_sensor, args=(sensor_index,))
        read_process.start()
        # notam timpul la care am masurat
        previous_time = current_time
        # incrementam indexul pentru a masura cu urmatorul senzor data viitoare
        sensor_index = sensor_index+1
        if sensor_index > 2:
            # intoarcem indexul la inceput dupa ce am masurat cu toti senzorii
            sensor_index = 0
    

def draw_lines(frame, position):
    # citire dimensiuni cadru de la webcam
    height, width, _ = frame.shape
    
    x_center = int(width/5)
    y_cutoff = int(0)
    
    # initializam un plan transparent pe care vom desena liniile
    transparent_plane = np.zeros((height, width, 4), dtype=np.uint8)
    
    # functie curba spre stanga
    def left_curve(y):
        return x_center/((height-(2*height-y_cutoff))*(height-y_cutoff))*(y-(2*height-y_cutoff))*(y-y_cutoff)

    # functie curba spre dreapta
    def right_curve(y):
        return 2*x_center - x_center/((height-(2*height-y_cutoff))*(height-y_cutoff))*(y-(2*height-y_cutoff))*(y-y_cutoff)

    # deseneaza de jos in sus curbele facand medie ponderata intre o linie dreapta si functia de curba in functie de pozitia volanului
    for y in range(height, y_cutoff, -1):
        x1 = 0
        x2 = 0
        
        if position > 0:
            # calculeaza curba spre stanga
            x1 = int(1*(1-abs(position))*x_center) + int(abs(position)*left_curve(y))
            x2 = int(1*(1-abs(position))*x_center) + int(abs(position)*left_curve(y))+int(3*x_center)
        else:
            # calculeaza curba spre dreapta
            x1 = int(1*(1-abs(position))*x_center) + int(abs(position)*right_curve(y))
            x2 = int(1*(1-abs(position))*x_center) + int(abs(position)*right_curve(y))+int(3*x_center)
        
        # deseneaza punctul curent din curba stanga si din curba dreapta pe planul transparent
        cv2.circle(transparent_plane, (x1, y), 4, (255, 0, 0, 255), -1)
        cv2.circle(transparent_plane, (x2, y), 4, (255, 0, 0, 255), -1)
    
    # deformam planul pentru a simula perspectiva
    src_points = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    dst_points = np.float32([[width * 0.2, height * 0.3], [width * 0.8, height * 0.3], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    transformed_plane = cv2.warpPerspective(transparent_plane, matrix, (width, height))

    # aplicam planul transparent cu liniile peste imaginea de la camera
    frame_bgra = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
    alpha_plane = transformed_plane[:, :, 3] / 255.0
    alpha_frame = 1.0 - alpha_plane
    for c in range(0, 3):
        frame_bgra[:, :, c] = (alpha_plane * transformed_plane[:, :, c] +
                               alpha_frame * frame_bgra[:, :, c])
    return frame_bgra
    
 
def read_potentiometer_position_from_channel(channel):
    # tensiune de la 0V la 3.3V
    voltage = channel.voltage
    
    # scalare tensiune de pe potentiometru in asa fel incat -1 dreapta si 1 stanga
    position = (voltage - 3.3/2)/1.65  
    
    # ne asiguram ca pozitia ramane in intervalul [-1; 1]
    if position > 1:
        position = 1
    elif position < -1:
        position = -1
        
    return position


if __name__ == '__main__':
    # initializeaza intrarile si iesirile rpi-ului
    init_gpio()
    
    # conexiunea i2c
    i2c = busio.I2C(board.SCL, board.SDA)
     
    # conexiune la ADC prin i2c
    ads = ADS.ADS1115(i2c)
     
    # selectarea canalului de intrare pentru adc
    channel = AnalogIn(ads, ADS.P0)

    #captura webcam
    cap = cv2.VideoCapture(0)

    while True:
        # citire imagine de pe webcam
        ret, frame = cap.read()
        
        # rotire imagine cu 180 grade
        rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        # citire pozitie potentiometru de la adc
        position = read_potentiometer_position_from_channel(channel)

        # desenare linii si distante
        new_frame = draw_lines(rotated_frame, position)
        
        # incercare masurare distanta
        attempt_distance_measurement()
        
        # desenare distante
        draw_distances(new_frame)
        
        # afisare imagine finala pe ecran
        cv2.imshow(window_name, new_frame)
        
        # daca s-a apasat tasta escape, inchide programul
        key = cv2.waitKey(20)
        if key == 27:
            break
    
    # inchidere fereastra
    cv2.destroyWindow(window_name)
    
    # inchidere captura webcam
    cap.release()
    
    # resetare stare GPIO
    GPIO.cleanup()
