import cv2
import numpy as np

#indeks 0 dla kamery w laptopie indeks 2 dla GoPro
cap=cv2.VideoCapture(0)

while True:
    _,frame=cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([135, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([5, 255, 255])

    lower_red2 = np.array([136, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Teraz możesz użyć tego jako obrazu wejściowego do wykrywania konturów
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    countours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)



    for cnt in contours_blue:
        if cv2.contourArea(cnt) < 300:  # Pomijamy szumy
            continue

        approx = cv2.approxPolyDP(cnt,0.04*cv2.arcLength(cnt,True),True)
        cv2.drawContours(frame, [approx], 0, 0, 3)
        print(len(approx))

        if len(approx)==3:
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            # rysujemy tekst
            # czcionka
            font = cv2.FONT_HERSHEY_SIMPLEX
            # co chcemy napisac
            frame = cv2.putText(frame, 'Trojkat ', (x, y), font, 1, (255, 0, 0), 4)

            #srodek ciezkosci trojkata
            srodek_tr_x = int((approx.ravel()[0] + approx.ravel()[2] + approx.ravel()[4]) / 3)
            srodek_tr_y = int((approx.ravel()[1] + approx.ravel()[3] + approx.ravel()[5]) / 3)
            cv2.circle(frame, (srodek_tr_x,srodek_tr_y), 3, (0, 255, 0), 3)

        if len(approx)==4:
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            # rysujemy tekst
            # czcionka
            font = cv2.FONT_HERSHEY_SIMPLEX
            # co chcemy napisac
            frame = cv2.putText(frame, 'Kwadrat ', (x, y), font, 1, (255, 0, 0), 4)



            #obliczanie srodka kwadratu
            srodek_kw_x=int((approx.ravel()[2]+approx.ravel()[6])/2)
            srodek_kw_y=int((approx.ravel()[3]+approx.ravel()[7])/2)
            cv2.circle(frame, (srodek_kw_x,srodek_kw_y), 3, (0, 255, 0), 3)
        if len(approx)==6:
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            # rysujemy tekst
            # czcionka
            font = cv2.FONT_HERSHEY_SIMPLEX
            # co chcemy napisac
            frame = cv2.putText(frame, 'Szesciokat ', (x, y), font, 1, (255, 0, 0), 4)

            #obliczanie srodka szesciokata
            srodek_sz_x = 0
            for i in range(0,12,2):
                srodek_sz_x = srodek_sz_x + approx.ravel()[i]
            srodek_sz_x = int(srodek_sz_x / 6)

            srodek_sz_y = 0
            for i in range(1, 12, 2):
                srodek_sz_y = srodek_sz_y + approx.ravel()[i]
            srodek_sz_y = int(srodek_sz_y / 6)

            cv2.circle(frame,(srodek_sz_x,srodek_sz_y), 3, (0, 255, 0), 3)

    for cnt in countours_red:
        if cv2.contourArea(cnt) < 300:  # Pomijamy szumy
            continue

        approx = cv2.approxPolyDP(cnt,0.04*cv2.arcLength(cnt,True),True)
        cv2.drawContours(frame, [approx], 0, 0, 3)
        print(len(approx))

        if len(approx)==3:
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            # rysujemy tekst
            # czcionka
            font = cv2.FONT_HERSHEY_SIMPLEX
            # co chcemy napisac
            frame = cv2.putText(frame, 'Trojkat ', (x, y), font, 1, (0, 0, 255), 4)

            # srodek ciezkosci trojkata
            srodek_tr_x = int((approx.ravel()[0] + approx.ravel()[2] + approx.ravel()[4]) / 3)
            srodek_tr_y = int((approx.ravel()[1] + approx.ravel()[3] + approx.ravel()[5]) / 3)
            cv2.circle(frame, (srodek_tr_x, srodek_tr_y), 3, (0, 255, 0), 3)

        if len(approx)==4:
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            # rysujemy tekst
            # czcionka
            font = cv2.FONT_HERSHEY_SIMPLEX
            # co chcemy napisac
            frame = cv2.putText(frame, 'Kwadrat ', (x, y), font, 1, (0, 0, 255), 4)

            # obliczanie srodka kwadratu
            srodek_kw_x = int((approx.ravel()[2] + approx.ravel()[6]) / 2)
            srodek_kw_y = int((approx.ravel()[3] + approx.ravel()[7]) / 2)
            cv2.circle(frame, (srodek_kw_x, srodek_kw_y), 3, (0, 255, 0), 3)

        if len(approx)==6:
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            # rysujemy tekst
            # czcionka
            font = cv2.FONT_HERSHEY_SIMPLEX
            # co chcemy napisac
            frame = cv2.putText(frame, 'Szesciokat', (x, y), font, 1, (0, 0, 255), 4)

            # obliczanie srodka szesciokata
            srodek_sz_x = 0
            for i in range(0, 12, 2):
                srodek_sz_x = srodek_sz_x + approx.ravel()[i]
            srodek_sz_x = int(srodek_sz_x / 6)

            srodek_sz_y = 0
            for i in range(1, 12, 2):
                srodek_sz_y = srodek_sz_y + approx.ravel()[i]
            srodek_sz_y =int(srodek_sz_y / 6)

            cv2.circle(frame, (srodek_sz_x, srodek_sz_y), 3, (0, 255, 0), 3)

    cv2.imshow('mask blue', mask_blue)
    cv2.imshow('mask red', mask_red)
    cv2.imshow('obraz', frame)


    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

