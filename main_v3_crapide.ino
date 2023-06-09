#include <Servo.h>
#include "Ultrasonic.h"
#include <Wire.h>
// defines pins numbers
const int in1 = 2;
const int in2 = 3;
const int in3 = 4;
const int in4 = 5;
#define cp_1 18
#define cm_1 16
#define cp_2 19
#define cm_2 17
#define PI 3.141592
#define nb_pas 1120
#define r_roue 30.4
#define fc_gauche 21
#define fc_start 15
#define sw_team 14
#define fc_droite 20
#define pin_servo_droit 7
#define pin_servo_gauche 6
#define capteur_avant 51
#define capteur_arriere 8
#define detec_p_g 22
#define detec_p_d 24
//define des objet et variable
Ultrasonic c_avant(capteur_avant);
Ultrasonic c_arriere(capteur_arriere);

Servo s_gauche;
Servo s_droit;
bool flag1 = false, flag2 = false;
double p_robot = 230;
int comp1 = 0;
double P_roue = PI * 2 * r_roue;                      //Définition & Calcul du périmètre dz la roue
double P_tour = PI * p_robot;                         //Définition & Calcul du périmètre du robot
float theta_tick = 360 * P_roue / (P_tour * nb_pas);  //Calcul la valeur du tick pour avoir parcourue 1rad
float dist_tick = P_roue / nb_pas;                    //Calcul la valeur du tick pour avoir parcourue 1mm*
double tck1, tck2, x_reel, y_reel, angle;
int state, lstate = digitalRead(fc_start);
bool flagstart = false;
bool team = digitalRead(sw_team);
long unsigned t1, t2, t3, ti;
int dist_de_secu = 12, points = 0;
int proche = 0;
bool flagstop = false, flagdecel = false;
bool secu_dist;
//proto:
void mouv_team_verte();
void mouv_team_bleu();
void servo(char x);
//interuption
void compteur1() {
  if (!digitalRead(cm_1)) {
    tck2--;
  } else {
    tck2++;
  }
}

void compteur2() {
  if (!digitalRead(cm_2)) {
    tck1--;
  } else {
    tck1++;
  }
}

void setup() {
  s_droit.attach(pin_servo_gauche);
  s_gauche.attach(pin_servo_droit);

  Serial.begin(115200);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(cp_1, INPUT_PULLUP);
  pinMode(cp_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(cp_1), compteur1, FALLING);
  attachInterrupt(digitalPinToInterrupt(cp_2), compteur2, RISING);


  servo('f');
  flag1 = false;
  flag2 = false;
  actu_points();

}

void loop() {

  Serial.println("aled");
  state = digitalRead(fc_start);
  if (state != lstate) {
    flagstart = true;
    ti = millis();
  }
  lstate = state;

  while (flagstart) {
      Serial.print("6\n");
    if (team) {
      //team bleu

      mouv_team_bleu();
    } else {
      // team verte

      mouv_team_verte();
    }
  }
}
void actu_points() {
  Serial.print("p");
  if (points < 10) {
    Serial.print("0");
    Serial.print(points);
  } else {
    Serial.print(points);
  }

  Serial.print("\n");
}
bool flag_end=false;
bool timmer() {
  unsigned long maxi = 95000;
  if (millis() - ti > maxi) {
    Serial.println((millis() - ti));
    digitalWrite(in2, LOW);
    analogWrite(in1, LOW);
    digitalWrite(in3, LOW);
    analogWrite(in4, LOW);
    flag_end=true;
    return false;
  } else {
    return true;
  }
}

bool timmer(unsigned long maxi) {
  maxi = maxi * 1000;
  if (millis() - ti > maxi) {
    digitalWrite(in2, LOW);
    analogWrite(in1, LOW);
    digitalWrite(in3, LOW);
    analogWrite(in4, LOW);
    Serial.println("aled");
    return false;
  } else {
    return true;
  }
}

void mouv_team_verte() {
  Serial.println("team vert");
  go_zone_verte();
  if (!digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 6;
  else {
    if (digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 3;
    if (!digitalRead(detec_p_d) && digitalRead(detec_p_g)) points += 3;
  }
  points += 3;


  avancer(150, -50);



  Serial.println("ici1");
  tourner(100, 90);  servo('f');
  Serial.println("ici2");
  flag1 = false;
  flag2 = false;
  while (!(flag1 && flag2) && timmer()) {
    if (digitalRead(fc_gauche)) {
      comp1++;
    } else {
      comp1 = 0;
    }
    if (comp1 < 15) {
      m_gauche('r', 70);
    } else {
      Serial.println("aled");
      m_gauche('s', 0);
      flag1 = true;
    }

    if (!digitalRead(fc_droite) && !flag2) {
      m_droite('r', 70);
    } else {
      flag2 = true;
      m_droite('s', 0);
    }
  }
  delay(500);
  avancer(250, 200);
  actu_points();
  angle = -90;
  x_reel = 2700;
  y_reel = 1750;
  servo('p');
  deplacement(1200, 1750, 250);
  if (!digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 6;
  else {
    if (digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 3;
    if (!digitalRead(detec_p_d) && digitalRead(detec_p_g)) points += 3;
  }
  actu_points();
  avancer(250, -200);

  angle = 180;
  x_reel = 1350;
  y_reel = 1750;
  bool rap = true;
  if (!rap) {
    servo('p');
    deplacement(1400, 900, 250);
    avancer(250, -200);
    y_reel = 1100;
    servo('f');
  } else {
    servo('f');


    tourner(150, -57);
        while (millis() - ti < 80000) {
      //coucou
    }

    avancer(250, 1000);
    x_reel = 600;
    y_reel = 600;
    angle = -90 - 45;

    dist_de_secu = 0;
    avancer(140, 600);
    if (!flag_end) {
      points += 15;
    }
    actu_points();delay(500);Serial.print("7\n");
    delay(100000000);
  }
}

void mouv_team_bleu() {
  Serial.println("team bleu");
  go_zone_bleu();
  if (!digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 6;
  else {
    if (digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 3;
    if (!digitalRead(detec_p_d) && digitalRead(detec_p_g)) points += 3;
  }
  points += 3;

  servo('o');


  avancer(250, -50);


  servo('f');
  Serial.println("ici1");
  tourner(100, -90);
  Serial.println("ici2");
  flag1 = false;
  flag2 = false;
  while (!(flag1 && flag2) && timmer()) {
    if (digitalRead(fc_gauche)) {
      comp1++;
    } else {
      comp1 = 0;
    }
    if (comp1 < 15) {
      m_gauche('r', 70);
    } else {
      Serial.println("aled");
      m_gauche('s', 0);
      flag1 = true;
    }

    if (!digitalRead(fc_droite) && !flag2) {
      m_droite('r', 70);
    } else {
      flag2 = true;
      m_droite('s', 0);
    }
  }
  delay(1000);
  avancer(250, 200);

  angle = 90;
  x_reel = 2700;
  y_reel = 250;
  servo('p');
  deplacement(1200, 250, 250);
  actu_points();
  if (!digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 6;
  else {
    if (digitalRead(detec_p_d) && !digitalRead(detec_p_g)) points += 3;
    if (!digitalRead(detec_p_d) && digitalRead(detec_p_g)) points += 3;
  }

  avancer(250, -200);
  actu_points();

  x_reel = 1400;
  y_reel = 250;
  servo('f');
   while (millis() - ti < 80000) {
    //coucou
  }
  deplacement(600, 1400, 250);
 
  dist_de_secu = 0;
  deplacement(500, 1550, 120);
  if (!flag_end) {
    points += 15;
  }
  actu_points();delay(500);Serial.print("7\n");
  delay(150000);
}

void grab(int x) {
  if (timmer(80)) {
    switch (x) {
      case 1:
        Serial.print("h0050\n");
        delay(2000);
        Serial.println("0\n");
        delay(300);
        Serial.print("h1400\n");
        delay(2000);
        break;
      case 2:
        Serial.print("h0600\n");
        delay(1300);
        Serial.println("0\n");
        delay(300);
        Serial.print("h1400\n");
        delay(2000);
        break;
      case 3:
        Serial.print("h1200\n");
        delay(1200);
        Serial.println("0\n");
        delay(300);
        Serial.print("h2000\n");
        delay(1000);
        break;
    }
  }
}

void releas(int x) {
  if (timmer(80)) {
    switch (x) {
      case 1:
        Serial.print("h0100\n");
        delay(2500);
        Serial.println("1\n");
        delay(200);
        Serial.print("h1400\n");
        delay(2000);
        break;
      case 2:
        Serial.print("h0700\n");
        delay(1500);
        Serial.println("1\n");
        delay(200);
        Serial.print("h1400\n");
        delay(2000);
        break;
      case 3:
        Serial.print("h1300\n");
        delay(1000);
        Serial.println("1\n");
        delay(200);
        Serial.print("h1500\n");
        delay(2000);
        break;
    }
  }
}
void go_zone_bleu() {
  servo('p');
  x_reel = 1230;
  y_reel = 390;
  angle = 90;
  deplacement(1250, 790, 250);
  deplacement(1810, 790, 250);

  deplacement(2000, 280, 250);
  deplacement(2650, 280, 250);
  servo('o');
}

void go_zone_verte() {
  servo('p');
  x_reel = 1230;
  y_reel = 1610;
  angle = -90;
  deplacement(1230, 1210, 250);
  deplacement(1880, 1210, 250);

  deplacement(2000, 1750, 250);
  deplacement(2650, 1750, 250);
  servo('o');
}

void servo(char x) {
  switch (x) {
    case 'f':
      s_gauche.write(360);
      s_droit.write(0);
      Serial.println("fermer");
      break;
    case 'o':
      s_gauche.write(0);
      s_droit.write(360);
      Serial.println("ouvert");
      break;
    case 'p':
      s_gauche.write(30);
      s_droit.write(149);

      break;
    case 'x':
      s_gauche.write(150);
      s_droit.write(30);

      break;
    default:
      break;
  }
}

void deplacement(int x, int y, int vitesse) {
  /*
  x,y=0,0 en bas a gauche; 0° regarde vers x+: en bas a gauche= zonne verte;

  */
  double a = x - x_reel, b = y - y_reel, c = sqrt(a * a + b * b);
  double angle_cible = atan(b / a) * 180 / PI, angle_d = 0;
  x_reel = x;
  y_reel = y;
  if (a >= 0) {
    if (b >= 0) {
      angle_cible = angle_cible;
      angle_d = angle_cible - angle;
      angle = angle_cible;
    } else {
      angle_cible = 360 + angle_cible;
      angle_d = angle_cible - angle;
      angle = angle_cible;
    }
  } else {
    if (b >= 0) {
      angle_cible = 180 + angle_cible;
      angle_d = angle_cible - angle;
      angle = angle_cible;
    } else {
      angle_cible = 270 - angle_cible;
      angle_d = angle_cible - angle;
      angle = angle_cible;
    }
  }
  if (angle_d < -180) {
    angle_d = angle_d + 360;
  } else {
    if (angle_d > 180) {
      angle_d = angle_d - 360;
    }
  }
  if (angle_d > 1 || angle_d < -1) {
    tourner(100, -angle_d);
  }
  delay(50);
  avancer(vitesse, c);
  delay(50);
}

void debug() {
  Serial.println("reele:");
  Serial.println(tck1);
  Serial.println("erreur:");
  Serial.println(abs(tck1) - abs(tck2));
}

void m_droite(char s, int vitesse) {
  if (timmer()) {
    switch (s) {
      case 'a':
        digitalWrite(in1, LOW);
        analogWrite(in2, vitesse);
        break;
      case 'r':
        digitalWrite(in2, LOW);
        analogWrite(in1, vitesse);
        break;
      case 's':
        digitalWrite(in2, LOW);
        analogWrite(in1, LOW);
        break;
      default:
        break;
    }
  }
}
void m_gauche(char s, int vitesse) {
  if (timmer()) {
    switch (s) {
      case 'a':
        digitalWrite(in4, LOW);
        analogWrite(in3, vitesse);
        break;
      case 'r':
        digitalWrite(in3, LOW);
        analogWrite(in4, vitesse);
        break;
      case 's':
        digitalWrite(in3, LOW);
        analogWrite(in4, LOW);
        break;
      default:
        break;
    }
  }
}
/*

    Commencez avec une valeur de Kp relativement élevée et Ki à zéro. Ensuite, ajustez Kp jusqu'à ce que le système réponde de manière stable à une entrée de référence.
Ensuite, ajustez Ki jusqu'à ce que l'erreur à long terme (offset) soit minimisée. Cependant, trop augmenter Ki peut causer une oscillation indésirable, donc il faut trouver un bon équilibre.
Si le système montre des oscillations indésirables, diminuez Kp et augmentez Ki. Si la réponse est trop lente, augmentez Kp et diminuez Ki.
Il est important de noter que les valeurs de Ki et Kp peuvent changer si le système est modifié, donc il faut réajuster régulièrement pour s'assurer des bonnes performances.


    */
double pas_stop;
bool flagaccel = false;
float calcule_vitesse(double pas, double pas_cible, int vitesse) {

  double deceleration = 255;
  double acceleration = 30;


  if (flagdecel) {
    deceleration = ((pas_stop + 1000) - pas) * (0.2) + 30;
    acceleration = 250;
  }

  if (!flagstop) {
    acceleration = pas * 0.2 + 35;
    if (pas_cible - pas < 500) {
      deceleration = (pas_cible - pas) * (0.2) + 30;
    } else {
      deceleration = 255;
    }

    if (flagaccel) {
      acceleration = (pas - pas_stop) * (0.2) + 30;
    }
    if (pas_stop + 500 < pas && flagaccel) {
      flagaccel = false;
      acceleration = 300;
    }
  }

  if (pas_stop + 800 < pas && flagstop) {
    flagdecel = false;
    vitesse = 0;
    deceleration = 300;
  }



  return (min(min(acceleration, vitesse), min(deceleration, vitesse)));
}

float calcule_vitesse2(double pas, double pas_cible, int vitesse) {
  double deceleration = -233 * pas / pas_cible + 263;
  double acceleration = 500 * pas / pas_cible + 30;

  return (min(min(acceleration, vitesse), min(deceleration, vitesse)));
}

bool secu_arret(char sens) {


  if (sens == 'r') {

    if (c_arriere.MeasureInCentimeters() > dist_de_secu) {
      if (flagstop) {
        delay(2000);
        flagaccel = true;
        pas_stop = abs(tck1);
      }
      flagstop = false;
      proche = 0;
      return true;
    } else {

      proche++;
      if (proche > 5) {
        if (!flagdecel) {
          pas_stop = abs(tck1);
          flagdecel = true;
        }
        flagstop = true;

        return false;
      }
    }
  } else {

    if (c_avant.MeasureInCentimeters() > dist_de_secu) {
      if (flagstop) {
        delay(2000);
        flagaccel = true;
        pas_stop = abs(tck1);
      }
      flagstop = false;
      proche = 0;
      return true;

    } else {
      proche++;
      if (proche > 5) {
        if (!flagdecel) {
          pas_stop = abs(tck1);
          flagdecel = true;
        }
        flagstop = true;
        return false;
      }
    }
  }
}

void avancer(double v_max, int l) {
  tck1 = 0;
  tck2 = 0;
  double vitesse_m1, vitesse_m2, vitesse;
  double flag1 = false, flag2 = false;

  if (l > 0)
    l = l - 0;
  else l = l + 0;
  double pas = l / dist_tick;



  char sens, nsens;
  double min_speed = v_max * 0.15, max_speed = v_max;
  double correction = 0, error, errorp, error_sum = 0;
  float Ki = 0.001, Kp = 1, Kd = 0;
  bool fr1 = false, fr2 = false;
  int comp = 10;
  if (pas < 0) {
    sens = 'r';
    nsens = 'a';
  } else {
    sens = 'a';
    nsens = 'r';
  }

  while (!(flag1 && flag2) && timmer()) {
    secu_dist = secu_arret(sens);

    vitesse = calcule_vitesse(abs(tck1), abs(pas), v_max);
    Serial.print("9\n");
    error = abs(tck2) - abs(tck1);
    error_sum += error;
    correction = Kp * error + Ki * error_sum + Kd * (error - errorp);
    errorp = error;

    //correction=max(-500,min(correction,500));

    vitesse_m1 = vitesse + correction;
    vitesse_m2 = vitesse - correction;

    vitesse_m1 = max(min_speed, min(max_speed, vitesse_m1));
    vitesse_m2 = max(min_speed, min(max_speed, vitesse_m2));
    if (vitesse = 0) {
      vitesse_m1 = 0;
      vitesse_m2 = 0;
    }
    //Serial.println(abs(tck2) - abs(tck1));
    if (secu_dist) {
      if (abs(tck1) < abs(pas) && !fr1 && !flag1) {
        m_gauche(sens, vitesse_m1);

      } else {
        if (abs(tck1) < abs(pas) + comp && !fr1) {
          m_gauche(sens, 30);
        } else {
          fr1 = true;
        }
        if (abs(tck1) > abs(pas) + comp && fr1) {
          m_gauche(nsens, 30);

        } else {
          if (fr1) {
            flag1 = true;
            m_gauche('s', 0);
          }
        }
      }

      if (abs(tck2) < abs(pas) && !fr2 && !flag2) {
        m_droite(sens, vitesse_m2);
      } else {
        if (abs(tck2) < abs(pas) + comp && !fr2) {
          m_droite(sens, 30);

        } else {
          fr2 = true;
        }
        if (abs(tck2) > abs(pas) + comp && fr2) {
          m_droite(nsens, 30);

        } else {
          if (fr2) {
            flag2 = true;
            m_droite('s', 0);
          }
        }
      }
    } else {
      m_gauche('s', 0);
      m_droite('s', 0);
    }
  }
  // delay(1000);
  // if (l > 0)
  //   l = l + 0;
  // else l = l - 0;
  // pas = l / dist_tick;
  // Serial.println("\n\n\npas theo");
  // Serial.println(pas);
  // Serial.println("pas réel");
  // Serial.println(tck1);
  // Serial.println("erreur");
  // Serial.println(abs(tck1) - abs(tck2));
  // Serial.println("erreure de pos");
  // Serial.println(((double)abs(tck1) - (double)abs(pas)) / (double)abs(pas));
}

void tourner(int v_max, int angle) {
  int theo1 = angle;

  p_robot = 207.3;
  tck1 = 0;
  tck2 = 0;
  double P_tour = PI * p_robot;  //Définition & Calcul du périmètre du robot
  float theta_tick = 360 * P_roue / (P_tour * nb_pas);
  double pas = angle / theta_tick;  //Calcul du nombre de tick pour tourné en fonction de l'angle voulu

  int comp = 10;
  double vitesse_m1, vitesse_m2, vitesse;
  double flag1 = false, flag2 = false;
  bool fr1 = false, fr2 = false;
  char sens1, sens2;
  double min_speed = v_max * 0.15, max_speed = v_max;
  double correction, error, error_sum = 0;
  float Ki = 0.00, Kp = 5;
  if (pas < 0) {
    sens1 = 'a';
    sens2 = 'r';
  } else {
    sens1 = 'r';
    sens2 = 'a';
  }

  while (!(flag1 && flag2) && timmer()) {
    vitesse = calcule_vitesse2(abs(tck1), abs(pas), v_max);
    Serial.print("9\n");
    error = abs(tck2) - abs(tck1);
    error_sum += error;
    correction = Kp * error + Ki * error_sum;

    vitesse_m1 = vitesse + correction;
    vitesse_m2 = vitesse - correction;
    vitesse_m1 = max(min_speed, min(max_speed, vitesse_m1));
    vitesse_m2 = max(min_speed, min(max_speed, vitesse_m2));

    //Serial.println(vitesse);
    //Serial.println(abs(tck2) - abs(tck1));
    if (abs(tck1) < abs(pas) && !fr1 && !flag1) {
      m_gauche(sens1, vitesse_m1);

    } else {
      if (abs(tck1) < abs(pas) + comp && !fr1) {
        m_gauche(sens1, 30);
      } else {
        fr1 = true;
      }
      if (abs(tck1) > abs(pas) + comp && fr1) {
        m_gauche(sens2, 40);

      } else {
        if (fr1) {
          flag1 = true;
          m_gauche('s', 0);
        }
      }
    }

    if (abs(tck2) < abs(pas) && !fr2 && !flag2) {
      m_droite(sens2, vitesse_m2);
    } else {
      if (abs(tck2) < abs(pas) + comp && !fr2) {
        m_droite(sens2, 30);

      } else {
        fr2 = true;
      }
      if (abs(tck2) > abs(pas) + comp && fr2) {
        m_droite(sens1, 40);

      } else {
        if (fr2) {
          flag2 = true;
          m_droite('s', 0);
        }
      }
    }
  }


  // delay(1000);

  // if (angle > 0)
  //   angle = angle + 0;
  // else angle = angle - 0;
  // pas = theo1 / theta_tick;
  // Serial.println("\n\n\npas theo");
  // Serial.println(pas);
  // Serial.println("pas réel");
  // Serial.println(tck1);
  // Serial.println("erreur");
  // Serial.println(abs(tck1) - abs(tck2));
  // Serial.println("erreure de pos");
  // Serial.println(((double)abs(tck1) - (double)abs(pas)) / (double)abs(pas));
}
