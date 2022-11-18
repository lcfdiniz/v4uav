#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from follower.srv import Detection,DetectionResponse

# Função para ordenar os ângulos das linhas, priorizando as linhas do topo
def ord_linhas(theta):
  return theta['max_y']

# Função que calcula o ponto de interseção entre duas linhas
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('As linhas não se interceptam')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

# Função que calcula a largura da linha na imagem, em pixels
def largura_linha(cv_image, box):
  x, y, w, h = box
  # Tratando valores fora dos limites da figura
  if x < 0:
    w = w - x
    x = 0
  if y < 0:
    h = h - y
    y = 0
  # Recorta o objeto da imagem
  objeto = cv_image[y:y+h, x:x+w]
  # Aplica diversos filtros no corte
  gray = cv2.cvtColor(objeto,cv2.COLOR_BGR2GRAY) # Converte para Grayscale
  blur_gray = cv2.GaussianBlur(gray,(5, 5),0) # Aplica um blur para elimar ruídos
  edges = cv2.Canny(blur_gray, 75, 150) # Identifica contornos
  kernel = np.ones((3,3), np.uint8)
  dilate = cv2.dilate(edges, kernel, iterations=1) # Dilata as linhas para facilitar a identificação

  # Identifica as linhas presentes no corte
  lines = cv2.HoughLinesP(dilate, 1, np.pi/180, 50, None, 50, 10)
  linha_central = [[0,h/2],[w,h/2]]
  maior_x = 0
  menor_x = w
  if type(lines) == np.ndarray:
    for line in lines:
      x1 = line[0][0]
      y1 = line[0][1]
      x2 = line[0][2]
      y2 = line[0][3]
      linha_aux = [[x1,y1],[x2,y2]]
      xi, yi = line_intersection(linha_central, linha_aux)
      if xi > maior_x:
        maior_x = xi
      elif xi < menor_x:
        menor_x = xi
  
  largura = maior_x - menor_x

  return x, y, w, h, largura

# Função que calcula o ângulo da linha identificada em relação ao drone
def calc_angulo(cv_image, box):
  x, y, w, h = box
  # Tratando valores fora dos limites da figura
  if x < 0:
    w = w - x
    x = 0
  if y < 0:
    h = h - y
    y = 0
  # Recorta o objeto da imagem
  objeto = cv_image[y:y+h, x:x+w]
  # Aplica diversos filtros no corte
  gray = cv2.cvtColor(objeto,cv2.COLOR_BGR2GRAY) # Converte para Grayscale
  blur_gray = cv2.GaussianBlur(gray,(5, 5),0) # Aplica um blur para elimar ruídos
  edges = cv2.Canny(blur_gray, 75, 150) # Identifica contornos

  # Identifica as linhas presentes no corte
  lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 50, 10)
  theta_med = 0.0
  thetas = []
  if type(lines) == np.ndarray:
    for line in lines:
      x1 = line[0][0]
      y1 = line[0][1]
      x2 = line[0][2]
      y2 = line[0][3]
      comp_x = abs(x2 - x1)
      comp_y = abs(y2 - y1)
      # Garante que a linha tem um comprimento mínimo, para eliminar ruídos no resultado
      if max(comp_x, comp_y) > 0.8*min(h, w):
        comps = [x2 - x1, y2 - y1]
        sinal = sum(n < 0 for n in comps)
        # Drone centralizado: 0 graus. Qualquer ângulo para a esquerda é positivo, e para a direita é negativo
        if sinal > 0:
          sinal = -1
        else:
          sinal = 1
        comp_x = float(comp_x)
        comp_y = float(comp_y)
        if comp_y == 0.0:
          thetas.append(0.0)
        else:
          theta = sinal*np.arctan(comp_x/comp_y)
          thetas.append(theta)

    # Garante que o número de linhas detectadas é maior que zero
    if len(thetas) > 0:
      # Calcula o desvio padrão dos ângulos encontrados
      dp = np.std(thetas)
      # Elimina ângulos até que o desvio padrão do conjunto seja menor que 5 graus
      # A perspectiva da imagem distorce as linhas das extremidades em até 5 graus
      while dp > 5*np.pi/180:
        media = np.mean(thetas)
        indice = 0
        dmax = dp
        for i,theta in enumerate(thetas):
          d = abs(theta - media)
          if d > dmax:
           dmax = d
           indice = i
        thetas.pop(indice)
        dp = np.std(thetas)

      media = np.mean(thetas)
      # O ângulo final é a média dos ângulos remanescentes no conjunto
      theta_med = round(np.rad2deg(media),1)

  return x, y, w, h, theta_med


def deteccoes(detection, threshold, boxes, confidences, class_ids, width, height):
  scores = detection[5:]
  class_id = np.argmax(scores)
  confidence = scores[class_id]
  if confidence > threshold:
    # Objeto detectado
    center_x = int(detection[0] * width)
    center_y = int(detection[1] * height)
    w = int(detection[2] * width)
    h = int(detection[3] * height)

    # Coordenadas do retangulo
    x = int(center_x - w / 2)
    y = int(center_y - h / 2)

    boxes.append([x, y, w, h])
    confidences.append(float(confidence))
    class_ids.append(class_id)

  return boxes, confidences, class_ids

def blob_imagem(net, imagem, output_layers):
  blob = cv2.dnn.blobFromImage(imagem, 1 / 255.0, (416,416), swapRB = True, crop = False)
  net.setInput(blob)
  layer_outputs = net.forward(output_layers)
  
  return net, imagem, layer_outputs

class detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("/follower/output_image", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/hexa_grin/usb_cam/image_raw",Image, self.callback, queue_size=1, buff_size=2**24)
    self.cv_image = None
    self.ld = 0
    self.th = []
    self.rh = []
    self.dy = []

  def handle_detection(self, req):
    if req.mode == 1:
      self.full_line_mode(self.cv_image)
      return {'ld': self.ld,
              'th': self.th,
              'rh': self.rh,
              'dy': self.dy}
    
    elif req.mode == 2:
      self.measure_line_mode(self.cv_image)
      return {'ld': 0,
              'th': [0.0],
              'rh': self.rh,
              'dy': [0.0]}
    
    elif req.mode == 3:
      self.one_line_mode(self.cv_image)
      return {'ld': self.ld,
              'th': [0.0],
              'rh': [0.0],
              'dy': self.dy}

  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def full_line_mode(self,cv_image):
    height, width, channels = cv_image.shape
    if width > 0 and height > 0:
      # ALTERAR APENAS ESSES TRÊS CAMPOS
      labels_path = "/home/lucas/Documents/GRIn/PTLIR/YOLO/follower.names"
      weights_path = "/home/lucas/Documents/GRIn/PTLIR/YOLO/yolov3_training_final.weights"
      config_path = "/home/lucas/Documents/GRIn/PTLIR/YOLO/yolov3_testing.cfg"

      labels = open(labels_path).read().strip().split('\n')
      np.random.seed(42)
      colors = np.random.randint(0,255,size=(len(labels),3), dtype='uint8')

      net = cv2.dnn.readNet(weights_path, config_path)
      layer_names = net.getLayerNames()
      output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

      # Processando a imagem de entrada
      net, cv_image, outs = blob_imagem(net, cv_image, output_layers)

      # Definindo variáveis
      threshold = 0.5
      threshold_NMS = 0.3
      confidences = []
      boxes = []
      class_ids = []
      linhas_detectadas = 0
      lista_thetas = []
      alturas = []
      deslocamentos = []
      fonte = cv2.FONT_HERSHEY_SIMPLEX
        
      # Realizando a predição
      for out in outs:
          for detection in out:
            boxes, confidences, class_ids = deteccoes(detection, threshold, boxes, confidences, class_ids, width, height)

      # Aplicando a Non-Max Supression
      indexes = cv2.dnn.NMSBoxes(boxes, confidences, threshold, threshold_NMS)

      for i in range(len(boxes)):
          if i in indexes:
            # Calcula o ângulo da linha identificada, em relação ao drone
            x, y, w, h, theta_med = calc_angulo(cv_image, boxes[i])

            # Trata valores extremos: a diferença entre +90 e -90 graus é sutil
            if abs(theta_med) > 80.0:
              if (x+w/2) > width/2:
                theta_med = -1*abs(theta_med)
              else:
                theta_med = abs(theta_med)
            
            # Preenche a estrutura de ângulo (utilizado para priorizar linhas do topo)
            theta = {"theta_med": theta_med, "max_y": y+h}

            focal_length = 277.19 # Distância focal da câmera utilizada
            w_real = 11.7 # Largura real da linha de transmissão
            h_relativa = 0.0
            delta_relativo = 0.0
            # Verifica se o ângulo da linha é menor que 5 graus
            if abs(theta_med) < 5.0:
              # Calcula a altura relativa entre a linha e o drone, em metros
              h_relativa = w_real*focal_length/w

              # Calcula a distância da linha para o centro da imagem
              delta_relativo = (width/2-(x+w/2))*h_relativa/focal_length

            # Preenchendo mensagem ROS customizada
            linhas_detectadas +=1
            lista_thetas.append(theta)
            alturas.append(h_relativa)
            deslocamentos.append(delta_relativo)

            # Insere os bounding boxes
            color = [int(c) for c in colors[class_ids[i]]]
            label = str(labels[class_ids[i]])
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(cv_image, label, (x + 5, y + 25), fonte, 0.8, color, 2)

      # Ordena as linhas correntamente, priorizando as do topo
      thetas = []
      lista_thetas.sort(key=ord_linhas)
      for i,theta in enumerate(lista_thetas):
        thetas.append(theta['theta_med'])

      # Plota na imagem os ângulos das linhas identificadas
      thetas_plot = []
      for theta in thetas:
        thetas_plot.append(int(theta))
      texto = "Angulo(s) da(s) linha(s): " + str(thetas_plot) + " graus"
      cv2.putText(cv_image, texto, (10,height-10), fonte, 0.4, [255,255,255], 0, lineType=cv2.LINE_AA)

      # Preenchendo as informações de detecção
      self.ld = linhas_detectadas
      self.th = thetas
      self.rh = alturas
      self.dy = deslocamentos

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def measure_line_mode(self,cv_image):
    height, width, channels = cv_image.shape
    if width > 0 and height > 0:
      # ALTERAR APENAS ESSES TRES CAMPOS
      labels_path = "/home/lucas/Downloads/YOLO/follower.names"
      weights_path = "/home/lucas/Downloads/YOLO/yolov3_training_final.weights"
      config_path = "/home/lucas/Downloads/YOLO/yolov3_testing.cfg"

      labels = open(labels_path).read().strip().split('\n')
      np.random.seed(42)
      colors = np.random.randint(0,255,size=(len(labels),3), dtype='uint8')

      net = cv2.dnn.readNet(weights_path, config_path)
      layer_names = net.getLayerNames()
      output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

      # Processando a imagem de entrada
      net, cv_image, outs = blob_imagem(net, cv_image, output_layers)

      # Definindo variaveis
      threshold = 0.5
      threshold_NMS = 0.3
      confidences = []
      boxes = []
      class_ids = []
      alturas = []
      fonte = cv2.FONT_HERSHEY_SIMPLEX
        
      # Realizando a predicao
      for out in outs:
          for detection in out:
            boxes, confidences, class_ids = deteccoes(detection, threshold, boxes, confidences, class_ids, width, height)

      # Aplicando a Non-Max Supression
      indexes = cv2.dnn.NMSBoxes(boxes, confidences, threshold, threshold_NMS)

      for i in range(len(boxes)):
          if i in indexes:
            # Calcula a largura da linha na imagem, em pixels
            x, y, w, h, largura = largura_linha(cv_image, boxes[i])

            focal_length = 277.19 # Distância focal da câmera utilizada
            w_real = 11.7 # Largura real da linha de transmissão
            h_relativa = 0.0
            delta_relativo = 0.0

            # Calcula a altura relativa entre a linha e o drone, em metros
            h_relativa = w_real*focal_length/largura

            # Preenchendo o vetor de alturas relativas
            alturas.append(h_relativa)

      # Preenchendo as informações de detecção
      self.rh = alturas

  def one_line_mode(self,cv_image):
    height, width, channels = cv_image.shape
    if width > 0 and height > 0:
      # Aplica diversos filtros no corte
      gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY) # Converte para Grayscale
      blur_gray = cv2.GaussianBlur(gray,(5, 5),0) # Aplica um blur para elimar ruídos
      edges = cv2.Canny(blur_gray, 75, 150) # Identifica contornos
      lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 50, 10)
      self.ld = len(lines)
      if type(lines) == np.ndarray:
        maior = 0.0
        for line in lines:
          comp = abs(line[0][3] - line[0][1])
          if comp > maior:
            maior = comp
            center_x = (line[0][0] + line[0][2])/2
            # Calcula a distância da linha para o centro da imagem
            focal_length = 277.19 # Distância focal da câmera utilizada
            delta_relativo = ((width/2) - center_x)*5/focal_length # Neste ponto, o drone estará a aproximadamente 5 metros da linha
      
      # Preenchendo as informações de detecção
      self.dy = [delta_relativo]

def detection_server():
  det = detector()
  rospy.init_node('detection_server')
  s = rospy.Service('detection', Detection, det.handle_detection)
  rospy.loginfo("Pronto para execução")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    detection_server()