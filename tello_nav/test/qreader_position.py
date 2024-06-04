from qreader import QReader
import cv2
import time

# Create a QReader instance d
qreader = QReader()

# Utiliser la capture vidéo à partir d'une caméra (remplacez le 0 par l'indice de la caméra)
# ou d'un fichier vidéo (remplacez "path/to/video.mp4" par le chemin de votre fichier vidéo)
cap = cv2.VideoCapture(2)  # Utiliser la caméra par défaut, ou bien cap = cv2.VideoCapture("path/to/video.mp4")

# Paramètres pour atteindre 15 images par seconde
target_fps = 15
frame_delay = 1.0 / target_fps

while cap.isOpened():
    start_time = time.time()
    
    # Lire un cadre vidéo
    ret, frame = cap.read()

    if not ret:
        print("Erreur lors de la lecture de la vidéo.")
        break

    # Convertir le cadre de BGR à RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Utiliser la fonction detect_and_decode pour obtenir les données QR décodées
    detection_result = qreader.detect_and_decode(image=rgb_frame, return_detections=True)
    # print(detection_result[0])

    print(detection_result)

    # Afficher le cadre vidéo avec OpenCV
    cv2.imshow("Video", frame)
    
    # Attente pour atteindre le taux cible
    elapsed_time = time.time() - start_time
    if elapsed_time < frame_delay:
        time.sleep(frame_delay - elapsed_time)

    # Quitter si la touche 'q' est pressée
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()
