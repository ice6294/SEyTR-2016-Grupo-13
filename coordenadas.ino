/***********************************************
 *  ARDUINO         SEyTR                v2.5  *
 *                                             *
 *  Proyecto:    RoboTracing        GRUPO   4  *
 *  Autores:                                   *
 *     Germán Alonso Azqutia        GIS + GII  *
 *     Oscar Ballesteros Izquierdo  GIS + GII  *
 *     Luis León Gámez              GIS + GII  *
 **********************************************/

 
// <<<<<<<<<<<<<<<<< INCLUDES >>>>>>>>>>>>>>>>>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Time-master/Time.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_HMC5883_U.h"




// <<<<<<<<<<<<<<<<< DEFINES >>>>>>>>>>>>>>>>>>
// TOPES & MÍNIMOS
#define MAX 255
#define MIN 0

// MODES
#define MODE_MANUAL         0
#define MODE_ACCELEROMETER  1
#define MODE_COORDINATES    2
#define MODE_FREELANCE      3

// MOTORES
#define A 10   // Enable Pin for motor A      (analog) (LEFT)	 -> A0
#define B 11   // Enable Pin for motor B      (analog) (RIGHT) -> A1

#define A1 8   // Control pin 1 for motor A   (digital)
#define A2 9   // Control pin 2 for motor A   (digital)
#define B1 12  // Control pin 1 for motor B   (digital)
#define B2 13  // Control pin 2 for motor B   (digital)

// POTENCIA
#define DEFAULT_POWER 80

// SENSOR
#define IS 7   // Input  sensor ultrasónico    (digital) (echo)
#define OS 6   // Output sensor ultrasónico    (analog)  (trig)
#define MAX_WALL_DISTANCE 20	// Distancia máxima para detectar una pared

// BLUETOOTH
#define RX 2   // Read  Bluetooth (va al TX de HC-06)
#define TX 3   // Write Bluetooth (va al RX de HC-06)

// LED
#define RED 4  		// Led Red
#define GREEN A1	// Led Green
#define LASER A0	// Laser

// ZUMBADOR
#define CLAX 5

// COMPAS
#define DECLINATION 0.01745	// radians

// COORDENADAS
#define PERIMETER 0.3

// COMMANDS
#define CMD_AVANZAR 'w'
#define CMD_RETROCEDER 's'
#define CMD_GIRAR_IZQUIERDA 'a'
#define CMD_GIRAR_DERECHA 'd'
#define CMD_PARAR 'p'
#define CMD_IMPRIMIR 'i'
#define CMD_CLAXON 'c'
#define CMD_RETURN 'r'
#define CMD_RETURN_90 't'
#define CMD_ROTATE_TIME_UP 'u'
#define CMD_ROTATE_TIME_DOWN 'j'
#define CMD_POWER_UP '+'
#define CMD_POWER_DOWN '-'
#define CMD_INITIALIZE 'z'
#define CMD_LASER 'l'

#define CMD_TO_MANUAL					('0' + MODE_MANUAL) // caracter '0'
#define CMD_TO_ACCELEROMETER	('0' + MODE_ACCELEROMETER) // caracter '1'
#define CMD_TO_COORDINATES		('0' + MODE_COORDINATES) // caracter '2'
#define CMD_TO_FREELANCE			('0' + MODE_FREELANCE) // caracter '3'

#define CMD_MAX_LENGTH 10

//#define MAX_CMD_LENGTH 5

// MOVING DELAY
#define STOP_DELAY 500

struct timeval {
	time_t        t;    // seconds
	unsigned long msec; // milliseconds
};

struct pint {
	float x;
	float y;
};




// <<<<<<<<<<<<<<<<< VARIABLES >>>>>>>>>>>>>>>>>
SoftwareSerial control(RX, TX);  // bluetooth
Adafruit_HMC5883_Unified compas; // compás
sensors_event_t event;           // sensor

// MOTORES
float power;         // potencia motores
float power_left;    // potencia motor izquierdo (%)
float power_right;   // potencia motor derecho   (%)

// MODOS
int mode;          // modo
bool stopMode;     // para cambiar de modo
int auxMode;			 // modo auxiliar cuando se le hace stopMode

// MOVIMIENTO
bool wallstop;     // true: para si ve una pared
bool moving;       // true: en movimiento
bool moved_front;  // true: ha avanzado algo
bool moved_back;   // true: ha retrocedido algo
float dist;        // distancia avanzada/retrocedida

// CHECK WALL
float wall_time;   // tiempo regreso onda
bool wallStop;		 // true: para si hay una pared delante

// COMANDO
char cmd;          // comando char
char complex_cmd[CMD_MAX_LENGTH];  // comando largo parseado
int posCmd;				 // leyendo posición comando complejo

bool simple_pending_cmd;  // simple char command
bool complex_pending_cmd; // float number command

// ÁNGULOS
float init_angle;    // angulo inicio (grados)
float angle;         // angulo actual (grados)

// COORDENADAS
float cx;          // coordenada EJE X
float cy;          // coordenada EJE Y

float dx;					 // coordenada EJE X destino
float dy;					 // coordenada EJE Y destino


// CÓDIGO ÓSCAR <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float valor;
char buffer[10] = "";
int posicion = 0;
int velocidad = 0;
// END    ÓSCAR <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// ACCELEROMETER
float ovalue;

// TIEMPO GIRO 90º (sin brújula)
float rotate_time;

// DISTANCIA
timeval before;     // hh:mm:ss en el que comienza a avanzar/retroceder
timeval after;      // hh:mm:ss en el que deja de avanzar/retroceder

//MODO FREELANCE
int randomDir;			//Variable aleatoria que selescciona elmovimiento random a realizar (girar, delante, atras)


// <<<<<<<<<<<<<<<< SETUP >>>>>>>>>>>>>>>>
/************************************************************
 * setup:
 *		Inicializa todos los pines, variables y elementos.
 ***********************************************************/
void setup() {

	// SERIAL
	Serial.begin(9600);

	// BLUETOOTH
	control.begin(9600);

	// RUEDAS
	for (int i = 8 ; i<13 ; i++)  // Inicializamos los pines del motor
		pinMode(i, OUTPUT);

	// SENSOR
	pinMode(OS, OUTPUT); // activación del pin 6 como salida: para el pulso ultrasónico
	pinMode(IS, INPUT);  // activación del pin 7 como entrada: tiempo del rebote del ultrasonido

	// LEDs & LASER
	pinMode(RED, OUTPUT);  	 // led red
	pinMode(GREEN, OUTPUT);  // led red
	pinMode(LASER, OUTPUT);  // led red

	// COMPÁS
	while(!compas.begin()) {
		control.println("#ERROR: no se detecta HMC5883. Revisar conexiones. Sigo intentando ...");
		delay(1000);
	}
	displaySensorDetails();

	// CLAX
	pinMode(CLAX, OUTPUT);

	// VALORES (x defecto)
	mode = MODE_MANUAL;
	wallStop = false;
	rotate_time = 700;

	initializePower();
	initializeCoords();

	control.println("Initializing components ...");
	compas = Adafruit_HMC5883_Unified(45); // 45 es la id. Como lo conectamos a los pines analógicos A4 y A5, elegimos esa ID

	simple_pending_cmd = false;
	complex_pending_cmd = false;

	randomDir = 0;		//Por defecto, modo hacia delante
	
	control.println("SETUP done.");
}

/************************************************************
 * initializePower:
 *		Inicializa la potencia de los motores.
 ***********************************************************/
void initializePower() {
	power = DEFAULT_POWER;
	power_left = 1;
	power_right = 1;
}

/************************************************************
 * initializeCoords:
 *		Inicializa las coordenadas, el ángulo y los valores
 *		booleanos de movimiento a false.
 ***********************************************************/
void initializeCoords() {
  cx = 0;
  cy = 0;
  moving = false;
  moved_front = false;
  moved_back = false;
  getAngle();
  init_angle = angle;
}




// <<<<<<<<<<<<<<<< LOOP >>>>>>>>>>>>>>>>
/************************************************************
 * loop:
 *		Bucle principal
 *			1. Lectura pared
 *			2. Lectura cmd
 *			3. Selección de modo
 ***********************************************************/
void loop() {
	checkWall();
	readCommand();
	selectMode();
}




// <<<<<<<<<<<<<<<< MODOS >>>>>>>>>>>>>>>>
/************************************************************
 * selectMode:
 *		Ejecuta el modo actual
 ***********************************************************/
void selectMode() {
	switch(mode) {
		case MODE_MANUAL:
			executeCmdManual();
			break;
		case MODE_ACCELEROMETER:
			//executeCmdAccelerometer();
			oscar();
			break;
		case MODE_COORDINATES:
			executeCmdCoordinates();
			break;
		case MODE_FREELANCE:
			//executeCmdFreelance();
			freelance();
			break;
		default:
			control.println("Me aburro ...");
			mode = MODE_MANUAL;
			break;
	}
}




// <<<<<<<<<<<<<<<< COMANDOS >>>>>>>>>>>>>>>>
/************************************************************
 * readCommand:
 *		Guarda en cmd o en complex_cmd el comando pasado por
 *		bluetooth. Depende del comando guardado activará
 *		simple_pending_cmd o complex_pending_cmd.
 ***********************************************************/
void readCommand() {
	if (control.available()) {
		cmd = control.read();
		// COMPLEX COMMANDS
		if (cmd == '#') { 				// # <- ACCELEROMETER
			cmd = control.read();
			posCmd = 0;
			// Leemos el comando en sí
			while(cmd != '$' && posCmd < CMD_MAX_LENGTH) {
				complex_cmd[posCmd] =	cmd;
				cmd = control.read();
				posCmd++;
			}
			ovalue = (atof(complex_cmd) + 0.8);	// 0'8 desviación típica móvil
			
			clearBuffer();
			complex_pending_cmd = true;
		} else if (cmd == '&') {	// & <- COORDINATES
			cmd = control.read();
			posCmd = 0;
			/** 
			 *  TODO: AQUÍ HABRÍA IDO EL CÓDIGO PARA LEER COORDENADAS &<COORD_X>@<COORD_Y>$
			 */
			// Leemos el comando en sí (coord X)
			while(cmd != '@' && posCmd < CMD_MAX_LENGTH) {
				complex_cmd[posCmd] =	cmd;
				cmd = control.read();
				posCmd++;
			}
			dx = atof(complex_cmd);
			clearBuffer();
			posCmd = 0;
			// Leemos el comando en sí (coord Y)
			while(cmd != '$' && posCmd < CMD_MAX_LENGTH) {
				complex_cmd[posCmd] =	cmd;
				cmd = control.read();
				posCmd++;
			}
			dy = atof(complex_cmd);
			clearBuffer();
			complex_pending_cmd = true;
			control.println("((coordenadas destino))");
			control.print("(( x: ");
			control.print(dx);
			control.println(" ))");
			control.print("(( y: ");
			control.print(dy);
			control.println(" ))");
		}

		// SIMPLE COMMANDS
		else {
			control.print(">>>> Command received: ");
			control.println(cmd);
			simple_pending_cmd = true;
		}
	}
}

/************************************************************
 * SELF-MOVING
 * readCmdStop:
 *		Si estamos en modo movimiento automático permitimos parar
 *		y volver al modo manual mientras.
 ***********************************************************/
void readCmdStop() {
 	if (control.available()) {
 		cmd = control.read();
 		if(	cmd == CMD_PARAR) {
 			stopMode = true;
 		} else if (cmd == CMD_TO_MANUAL) {
 			stopMode = true;
 			auxMode = MODE_MANUAL;
 		} else if (cmd == CMD_TO_ACCELEROMETER ) {
 			stopMode = true;
 			stopMode = MODE_ACCELEROMETER;
 		} else if (cmd == CMD_TO_COORDINATES) {
 			stopMode = true;
 			stopMode = MODE_COORDINATES;
 		} else if (cmd == CMD_TO_FREELANCE ) {
 			stopMode = true;
 			stopMode = MODE_FREELANCE;
 		}
 	}
}

/************************************************************
 * MANUAL
 * executeCmdManual:
 *		Ejecutamos un comando simple o complejo en modo manual (0).
 ***********************************************************/
void executeCmdManual() {
  // SIMPLE CMD MAN
  if (simple_pending_cmd) {
    switch(cmd) {
      // AVANZAR
      case CMD_AVANZAR:
        avanzar();
        break;

      // RETROCEDER
      case CMD_RETROCEDER:
        retroceder();
        break;

      // IZQ
      case CMD_GIRAR_IZQUIERDA:
        girarIzquierda();
        break;

      // DER
      case CMD_GIRAR_DERECHA:
        girarDerecha();
        break;

      // STOP
      case CMD_PARAR:
        parar();
        break;

      // IMPRIMIR
      case CMD_IMPRIMIR:
        imprimir();
        break;

      // CLAXON
      case CMD_CLAXON:
        beep();
        break;

      // RETURN
      case CMD_RETURN:
      	mode = MODE_COORDINATES;
      	control.println("MODO >> COORDINATES (return to ZERO)");
        goTo(0,0);
        break;

			// RETURN 90
			case CMD_RETURN_90:
				mode = MODE_COORDINATES;
				control.println("MODO >> COORDINATES (return 90)");
        goToBy90(0,0);
        break;
      
      // INITIALIZE ALL
      case CMD_INITIALIZE:
        initializeCoords();
        break;

     	// SWITCH TO ACELEROMETER
      case CMD_TO_ACCELEROMETER:
      	mode = MODE_ACCELEROMETER;
      	control.println("MODO >> ACCELEROMETER");
      	parar();
      	break;

      // SWITCH TO COORDINATES
      case CMD_TO_COORDINATES:
      	mode = MODE_COORDINATES;
      	control.println("MODO >> COORDINATES");
      	parar();
      	break;

      // SWITCH TO FREELANCE
      case CMD_TO_FREELANCE:
      	mode = MODE_FREELANCE;
      	control.println("MODO >> FREELANCE");
      	parar();
      	break;

     	// ROTATE TIME UP
     	case CMD_ROTATE_TIME_UP:
     		rotate_time = rotate_time + 0.2;
     		break;

     	// ROTATE TIME DOWN
     	case CMD_ROTATE_TIME_DOWN:
     		rotate_time = rotate_time - 0.2;
     		break;

     	// POWER UP
			case CMD_POWER_UP:
				powerUp();
				break;
     	
     	// POWER DOWN
     	case CMD_POWER_DOWN:
				powerUp();
				break;
      	
      // DEFAULT
      default:
        break;
    }
    simple_pending_cmd = false;

  // COMPLEX CMD MAN
  } else if (complex_pending_cmd) {
    complex_pending_cmd= false;
  }
}

/************************************************************
 * ACCELEROMETER
 * executeCmdAccelerometer:
 *		Ejecutamos un comando simple o complejo en modo accelerometer (1).
 ***********************************************************/
void executeCmdAccelerometer() {
  // SIMPLE CMD AUT
  if (simple_pending_cmd) {
    switch(cmd) {
      // PARAR
      case CMD_PARAR:
        stopMode = true;
        parar();
        break;

      // SWITCH TO MANUAL
      case CMD_TO_MANUAL:
      	mode = MODE_MANUAL;
      	control.println("MODO >> MANUAL");
      	parar();
      	initializePower();
      	break;

      // SWITCH TO COORDINATES
      case CMD_TO_COORDINATES:
      	mode = MODE_COORDINATES;
      	control.println("MODO >> COORDINATES");
      	parar();
      	break;

      // SWITCH TO FREELANCE
      case CMD_TO_FREELANCE:
      	mode = MODE_FREELANCE;
      	control.println("MODO >> FREELANCE");
      	parar();
      	break;

      // POWER UP
			case CMD_POWER_UP:
				powerUp();
				avanzar();
				break;
     	
     	// POWER DOWN
     	case CMD_POWER_DOWN:
				powerUp();
				avanzar();
				break;

			// DEFAULT
			default:
				break;
    }
    simple_pending_cmd= false;

  // COMPLEX CMD MAN
  } else if (complex_pending_cmd) {
  	parseOvalue();	// actualizamos power_left y power_right
  	avanzar();			// actualizamos potencia motores
    complex_pending_cmd= false;
  }
}

/************************************************************
 * COORDINATES
 * executeCmdCoordinates:
 *		Ejecutamos un comando simple o complejo en modo coordenadas (2).
 ***********************************************************/
void executeCmdCoordinates() {
  // SIMPLE CMD ACC
  if (simple_pending_cmd) {
    switch(cmd) {
      // PARAR
      case CMD_PARAR:
        stopMode = true;
        parar();
        break;

      // SWITCH TO MANUAL
      case CMD_TO_MANUAL:
      	mode = MODE_MANUAL;
      	control.println("MODO >> MANUAL");
      	parar();
      	initializePower();
      	break;

			// SWITCH TO ACELEROMETER
      case CMD_TO_ACCELEROMETER:
      	mode = MODE_ACCELEROMETER;
      	control.println("MODO >> ACCELEROMETER");
      	parar();
      	break;

      // SWITCH TO FREELANCE
      case CMD_TO_FREELANCE:
      	mode = MODE_FREELANCE;
      	control.println("MODO >> FREELANCE");
      	parar();
      	break;

			// DEAFULT
      default:
        break;
    }
    simple_pending_cmd= false;

  // COMPLEX CMD MAN
  } else if (complex_pending_cmd) {
  	goTo(dx,dy);
    complex_pending_cmd= false;
  }
}

/************************************************************
 * FREELANCE
 * executeCmdFreelance:
 *		Ejecutamos un comando simple o complejo en modo freelance (3).
 ***********************************************************/
void executeCmdFreelance() {
  // SIMPLE CMD COR
  if (simple_pending_cmd) {
    switch(cmd) {
      // PARA
      case CMD_PARAR:
        stopMode = true;
        parar();
        break;

      // SWITCH TO MANUAL
      case CMD_TO_MANUAL:
      	mode = MODE_MANUAL;
      	control.println("MODO >> MANUAL");
      	parar();
      	initializePower();
      	break;

			// SWITCH TO ACELEROMETER
      case CMD_TO_ACCELEROMETER:
      	mode = MODE_ACCELEROMETER;
      	control.println("MODO >> ACCELEROMETER");
      	parar();
      	break;

      // SWITCH TO COORDINATES
      case CMD_TO_COORDINATES:
      	mode = MODE_COORDINATES;
      	control.println("MODO >> COORDINATES");
      	parar();
      	break;

      // DEAFULT
      default:
        break;
    }
    simple_pending_cmd= false;

  // COMPLEX CMD MAN
  } else if (complex_pending_cmd) {
    complex_pending_cmd= false;
  }
}




// <<<<<<<<<<<<<<<< CONTROL >>>>>>>>>>>>>>>>
/************************************************************
 * avanzar:
 *		Activa los motores hacia delante y activa el cronómetro.
 *		La potencia de los motores se regula desde las variables globales.
 ***********************************************************/
void avanzar() {
  // Si me movía hacia delante -> no hacer nada
  if(!(moving && moved_front)) {
	  // Si me movía hacia atrás   -> parar
	  if(moving && moved_back) {
	    parar(); // -> hace stop chrono
	  }
	  // Cogemos el ángulo y activamos moving y moved_front
	  getAngle();
	  moving = true;
	  moved_front = true;
	
	  // Arrancamos el cronómetro
	  startChrono();
  }

  // Activamos motores (hacia delante)
  analogWrite(A, power * power_left);		// Activamos MotorA
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);

  analogWrite(B, power * power_right);	// Activamos MotorB
  digitalWrite(B1, HIGH);
  digitalWrite(B2, LOW);
}

/************************************************************
 * retroceder:
 *		Activa los motores hacia atrás y activa el cronómetro.
 *		La potencia de los motores se regula desde las variables globales.
 ***********************************************************/
void retroceder() {
  // Si me movía hacia atrás   -> no hacer nada
  if (!(moving && moved_back)) {
	  // Si me movía hacia delante -> parar
	  if(moving && moved_front) {
	    parar(); // -> hace stop chrono
	  }
	  // Cogemos el ángulo y activamos moving y moved_back
	  getAngle();
	  moving = true;
	  moved_back = true;
	
		// Arrancamos el cronómetro
	  startChrono();
  }
  // Activamos motores (hacia atrás)
  analogWrite(A, power * power_left);		// Activamos MotorA
  digitalWrite(A1, LOW);
  digitalWrite(A2, HIGH);

  analogWrite(B, power * power_left);		// Activamos MotorB
  digitalWrite(B1, LOW);
  digitalWrite(B2, HIGH);
}

/************************************************************
 * girarDerecha:
 *		Activa los motores hacia la derecha.
 *		La potencia de los motores se regula desde las variables globales.
 ***********************************************************/
void girarDerecha() {
  // Si me movía -> parar
  if (moving)
    parar(); // -> hace stop chrono

  // Activamos motores (hacia derecha)
  analogWrite(A, power * power_left);		// Activamos MotorA
  digitalWrite(A1, HIGH);
  digitalWrite(A2, LOW);

  analogWrite(B, power * power_right);	// Activamos MotorB
  digitalWrite(B1, LOW);
  digitalWrite(B2, HIGH);
}

/************************************************************
 * girarIzquierda:
 *		Activa los motores hacia la izquierda.
 *		La potencia de los motores se regula desde las variables globales.
 ***********************************************************/
void girarIzquierda() {
  // Si me movía -> parar
  if (moving)
    parar(); // -> hace stop chrono

  // Activamos motores (hacia izquierda)
  analogWrite(A, power * power_left);		// Activamos MotorA
  digitalWrite(A1, LOW);        // Arrancamos
  digitalWrite(A2, HIGH);

  analogWrite(B, power * power_right);	// Activamos MotorB
  digitalWrite(B1, HIGH);
  digitalWrite(B2, LOW);
}

/************************************************************
 * parar:
 *		Para los motores, detiene el cronómetro y calcula las nuevas coordenadas.
 *		El STOP_DELAY es para simular la deceleración en la parada.
 ***********************************************************/
void parar() {
	// Paramos motores
  analogWrite(A, LOW);     // Paramos MotorA
  analogWrite(B, LOW);     // Paramos MotorB

  // Si me movía -> para cronómetro y calcular nuevas coordenadas
  if (moving) {
  	delay(STOP_DELAY);
  	stopChrono();
  	calculateNewCoords();
  }
  moving = false;
}




// <<<<<<<<<<<<<<<< COORDENADAS >>>>>>>>>>>>>>>>
/************************************************************
 * calculateNewCoords:
 *		Actualiza las coordenadas cx y cy cuando para.
 ***********************************************************/
void calculateNewCoords() {
  // Depende de para dónde se haya movido, actualizamos coordenadas
  if (moved_front) {
    cx -= dist * cos(toRadianes(angle)); // + ???
    cy -= dist * sin(toRadianes(angle)); // + ???
  } else if (moved_back) {
    cx += dist * cos(toRadianes(angle)); // - ???
    cy += dist * sin(toRadianes(angle)); // - ???
  }
  // Actualizamos a false, pues ya guardado la información
  dist = 0;
  moved_front = false;
  moved_back = false;
}

/************************************************************
 * calculateNewCoordsWhileAvanzing:
 *		Actualiza las coordenadas cx y cy mientras avanza.
 *		Usado en modo accelerometer y coordinates.
 ***********************************************************/
void calculateNewCoordsWhileAvanzing() {
  if(moving) {
    stopChrono();
    if (moved_front) {
      cx -= dist * cos(toRadianes(angle)); // + ???
      cy -= dist * sin(toRadianes(angle)); // + ???
    } else if (moved_back) {
      cx += dist * cos(toRadianes(angle)); // - ???
      cy += dist * sin(toRadianes(angle)); // - ???
    }
    startChrono();
  }
}

/************************************************************
 * getAngle:
 *		Guarda en la variable global angle la orientación actual del robot.
 ***********************************************************/
void getAngle() {
  // Recogemos evento compás
  compas.getEvent(&event);
  float sample = atan2(event.magnetic.y, event.magnetic.x);
  sample += DECLINATION;
  // Corrige los valores negativos
  if(sample < 0)
    sample += 2 * PI;
  // Comprueba si hay error debido a la adición de la declinación.
  if(sample > 2 * PI)
    sample -= 2 * PI;
  // Convierte los radianes a grados
  angle = toGrados(sample);
}





// <<<<<<<<<<<<<<<< TIEMPO >>>>>>>>>>>>>>>>
/************************************************************
 * startChrono:
 *		Arranca el cronómetro y lo guarda en la variable global before
 ***********************************************************/
void startChrono() {
  // Capturamos momento de antes
  //before = now(); <NEW>
  before.t = now();
  before.msec = millis();
}

/************************************************************
 * stopChrono:
 *		Para el cronómetro y lo guarda en la variable global after.
 *		Con before y after calcula la nueva distancia recorrida
 *		y lo guarda en la variable global dist.
 ***********************************************************/
void stopChrono() {
  if (moving) {
    // Capturamos momento de después
    after.t = now();
    after.msec = millis();
    // Convertimos a segundos el tiempo antes y después
    int before_int = minute(before.t) * 60 + second(before.t) + before.msec * 0.001;
    int after_int = minute(after.t) * 60 + second(after.t) + after.msec * 0.001;
    // Guardamos la distancia recorrida desde startChrono
    dist = before_int - after_int;
  }
}




// <<<<<<<<<<<<<<<< AUTO-CONTROL >>>>>>>>>>>>>>>>
/************************************************************
 * getAngleTo:
 *		\param[in]	new_x		· Coordenada x hacia donde tiene que mirar.
 *		\param[in]	new_y		· Coordenada y hacia donde tiene que mirar.
 *		\param[out]	float		· Angulo desde 0º hasta la nueva coordenada.
 ***********************************************************/
float getAngleTo(float new_x, float new_y) {
  // Obtenemos la ubicación del nuevo punto
  float diff_x = new_x - cx;
  float diff_y = new_y - cy;
  // Calculamos con el arcotangente y los dos catetos el angulo hacia el el nuevo punto
  float atanangle = toGrados(atan2(diff_y, diff_x));
  // Re-ajustamos el ángulo dependiendo del cuadrante (si está en el primero (+X,+Y) no es necesario)
  float angleToNewLocate  = atanangle;
  if(angleToNewLocate > 180) {
    // Si está en Y negativo, al 
    angleToNewLocate = 360 - atanangle;
  }
  // Quiero que me lo de en positivo
  while (angleToNewLocate < 0) {
    angleToNewLocate += 360;
  }
  while (angleToNewLocate > 360) {
    angleToNewLocate -= 360;
  }
  // Corregimos valores fuera de rango
  return angleToNewLocate;
}

/************************************************************
 * getDiff:
 *		\param[in]	angle_origin	· Ángulo origen 
 *		\param[in]	angle_dest		· Ángulo destino
 *		\param[out]	float					· Grados y dirección para girar
 *															desde el ángulo origen hacia el
 *															ángulo destino.
 *
 *		Giro a la izquierda -> POS +
 *		Giro a la derecha -> NEG -
 ***********************************************************/
float getDiff(float angle_origin, float angle_dest) {
  float aux = angle_dest - angle_origin;
  if (aux > 180) {
    aux -= 360;
  }
  if (aux < -180) {
    aux +=360;
  }
  return aux;
}

/************************************************************
 * COORDINATES
 * rotateTo:
 *		\param[in]	dest		· Ángulo hacia el que tiene que orientarse.
 *		
 *		Gira hasta que quede orientado hacia el ángulo destino.
 ***********************************************************/
void rotateTo(float dest) {
  // Mientras no esté orientado hacia el ángulo destino
  float diff = getDiff(angle, dest);
  bool exito = fabsf(diff) < 10; // margen de 10º con el ángulo destino;
  while(!exito && !stopMode) {
  	control.print("    . agle:");
  	control.println(angle);
  	control.print("    . dest:");
  	control.println(dest);
    control.print("        . diff = ");
    control.print(diff);
    // Toma el camino más rápido para girar
    if(diff > 0) {
      control.println(" -> der");
      girarDerecha(); // Giramos hacia la derecha
    } else {
      control.println(" -> izq");
      girarIzquierda(); // Giramos hacia la izquierda
    }
    getAngle();
    diff = getDiff(angle, dest);
    // Sale del bucle si está orientado hacia el destino
    exito = fabsf(diff) < 10;
    readCmdStop();
  }
}

/************************************************************
 * getAngleTo:
 *		\param[in]	new_x		· Coordenada x en la que tiene que estar el robot
 *		\param[in]	new_y		· Coordenada y en la que tiene que estar el robot
 *		\param[out]	bool		· True si está en esas coordenadas.
 ***********************************************************/
bool inEnd(float end_x, float end_y) {
  return ( cx < (end_x + PERIMETER)
        && cx > (end_x - PERIMETER)
        && cy < (end_y + PERIMETER)
        && cy > (end_y - PERIMETER) );
}

/************************************************************
 * goTo:
 *		\param[in]	new_x		· Coordenada x a la que tiene que ir
 *		\param[in]	new_y		· Coordenada y a la que tiene que ir
 ***********************************************************/
void goTo(float end_x, float end_y) {
	// Mientras no haya llegado a casa
  float end_angle;
  bool exito = inEnd(end_x, end_y);
  stopMode = false;
  while(!exito && !stopMode) {
  	// Calcula el ángulo destino desde las coordenadas actuales
  	end_angle = getAngleTo(end_x, end_y);
    // Rotamos hacia el destino (si no apuntamos ya hacia allí)
    rotateTo(end_angle);
    // Avanza hacia allí
    avanzar();
    control.println("Avanzo ...");
    control.print("    x: ");
    control.println(cx);
    control.print("    y: ");
    control.println(cy);
    calculateNewCoordsWhileAvanzing();
    // exito -> ha llegado
    exito = inEnd(end_x, end_y);
    readCmdStop();
  }
  // Cuando haya llegado/lo hayan parado -> para
  if (exito) {
    rotateTo(init_angle);
    success();
  } else {
    fail();
  }
  if (stopMode) {
  	mode = auxMode;
  	auxMode = MODE_MANUAL;
  	stopMode = false;
  }
}




// <<<<<<<<<<<<<<<< AUTO-CONTROL-90º >>>>>>>>>>>>>>>>
/************************************************************
 * goToBy90:
 *		Se dirige hacia un punto mediante giros de 90º
 ***********************************************************/
void goToBy90(float new_x, float new_y) {
	// Nos dirigimos al eje Y
	if (cx > new_x) {
		rotateToSouth();
	} else {
		rotateToNorth();
	}
	// Avanzamos hasta alcanzar el eje Y del nuevo punto
	while(abs(cx - new_x) > PERIMETER && !stopMode) {
		readCmdStop();
		avanzar();
		calculateNewCoordsWhileAvanzing();
	}
	// Nos dirigimos al eje X
	if (cy > new_y) {
		rotateToEast();
	} else {
		rotateToWest();
	}
	// Avanzamos hasta alcanzar el eje X del nuevo punto
	while(abs(cy - new_y) > PERIMETER && !stopMode) {
		readCmdStop();
		avanzar();
		calculateNewCoordsWhileAvanzing();
	}
	success();
	if (stopMode) {
  	mode = auxMode;
  	auxMode = MODE_MANUAL;
  	stopMode = false;
  }
	
}

/************************************************************
 * rotateToNorth:
 *		Se horienta hacia el norte
 ***********************************************************/
void rotateToNorth() {
	float dest = 0;	// angle to North
	rotateTo(dest);
}

/************************************************************
 * rotateToWest:
 *		Se horienta hacia el norte
 ***********************************************************/
void rotateToWest() {
	float dest = 90;	// angle to West
	rotateTo(dest);
}

/************************************************************
 * rotateToSouth:
 *		Se horienta hacia el norte
 ***********************************************************/
void rotateToSouth() {
	float dest = 180;	// angle South
	rotateTo(dest);
}

/************************************************************
 * rotateToEast:
 *		Se horienta hacia el norte
 ***********************************************************/
void rotateToEast() {
	float dest = 270; // angle East
	rotateTo(dest);
}




// <<<<<<<<<<<<<<<< FREELANCE >>>>>>>>>>>>>>>>
/************************************************************
 * rotate90Izq:
 *		Gira sin brújula 90º aprox. hacia la izquierda
 ***********************************************************/
void rotate90Izq() {
	control.println("$ Giro hacia la IZQUIERDA 90º $");
	girarIzquierda();
	delay(rotate_time);
	parar();
}

/************************************************************
 * rotate90Der:
 *		Gira sin brújula 90º aprox. hacia la derecha
 ***********************************************************/
void rotate90Der() {
	control.println("$ Giro hacia la DERECHA 90º $");
	girarDerecha();
	delay(rotate_time);
	parar();
}

/************************************************************
 * avanzarRandom:
 *		Avanza hacia delante durante un tiempo aleatorio (1-4 seg)
 ***********************************************************/
void avanzarRandom() {
	randomDir = (rand() % 4) + 1; // de 1 a 4 (segundos que se va a mover)
	unsigned long bef = millis();
	unsigned long aft = millis();
	avanzar();
	bool exit1 = checkWall();
	bool exit2 = ((bef - aft) > (randomDir * 1000));
	while(!exit1 && !exit2) {	// OJO!!!!
		aft = millis();
		exit1 = checkWall();
		exit2 = ((bef - aft) > (randomDir * 1000));
	}
	parar();
	if (exit1) {
		rotate90Izq(); // mira a ver si ha algo a la izquierda. Si no lo hay se sale
		delay(200);
		if (checkWall) {
			rotate90Der();	// mira a ver si ha algo a la derecha. Si no lo hay se sale
			delay(200);
			rotate90Der();
			delay(200);
			if (checkWall) {	// si hay algo a ambos lados, que vuelva por donde ha venido (calle sin salida)
				rotate90Der();
				avanzar();
				delay(bef-aft);
				parar();
			}
		}
	} else if (exit2) {
		randomDir = rand() % 3;
		switch(randomDir) {
			// BEEP BEEP
			case 0:
				beep();
				delay(100);
				beep();
				break;
			// HEAD SHOOT
			case 1:
				ledOn(LASER);
				delay(500);
				ledOff(LASER);
				break;
			// LIGHT PARTY
			case 2:
				ledOn(RED);
				delay(500);
				ledOff(RED);
				ledOn(GREEN);
				delay(500);
				ledOff(GREEN);
				break;
			default:
				break;
		}
	}
}

/************************************************************
 * freelance:
 *		Movimiento libre autónomo (evitando chocarse con paredes)
 *		TODO: aleatorizar comportamiento robot
 ***********************************************************/
void freelance() {
	while(!stopMode) {
		readCmdStop();
		randomDir = rand() % 2;
		switch(randomDir) {
			case 0: //Hacia delante
				avanzarRandom();				
				break; 
			case 1:	//Girar
					randomDir = rand() % 2;
					if(randomDir == 0){ // Girar derecha
						rotate90Der();
					}else{ //Girar Izquierda
						rotate90Izq();
					}
				break;
			default:
				control.println("Me aburro ...");
				break;
		}
		delay(500);
		
	} //Fin while
	if (stopMode) {
  	mode = auxMode;
  	auxMode = MODE_MANUAL;
  	stopMode = false;
  }
}



// <<<<<<<<<<<<<<<< IMPRIMIR >>>>>>>>>>>>>>>>
/************************************************************
 * displaySensorDetails:
 *		Imprime los datos del compás
 ***********************************************************/
void displaySensorDetails() {
  sensor_t sensor;
  compas.getSensor(&sensor);
  control.println("----------------------------------------------");
  control.print  ("Sensor:       "); control.println(sensor.name);
  control.print  ("Driver Ver:   "); control.println(sensor.version);
  control.print  ("ID unica:    "); control.println(sensor.sensor_id);
  control.print  ("Valor Maximo:    "); control.print(sensor.max_value); control.println(" uT");
  control.print  ("Valor Minimo:    "); control.print(sensor.min_value); control.println(" uT");
  control.print  ("Resolucion:   "); control.print(sensor.resolution); control.println(" uT");  
  control.println("----------------------------------------------");
  control.println("");
  control.flush();
}

/************************************************************
 * imprimir:
 *		Imprime los datos datos de la ubicación actual
 ***********************************************************/
void imprimir() {
  control.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  control.print("  x: ");
  control.println(cx);
  control.print("  y: ");
  control.println(cy);
  control.print("  actual angle: ");
  getAngle();
  control.println(angle);
  float aux = getAngleTo(0,0);
  control.print("  angle to (0,0): ");
  control.println(aux);
  control.print("  diff: ");
  float aux2 = getDiff(angle, aux);
  control.println(aux2);
  control.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  control.flush();
}

/************************************************************
 * success:
 *		Imprime éxito
 ***********************************************************/
void success() {
  control.println("#######################");
  control.println("######## EXITO ########");
  control.println("#######################");
}

/************************************************************
 * success:
 *		Imprime fallo
 ***********************************************************/
void fail() {
  control.println("#######################");
  control.println("######### :( ##########");
  control.println("#######################");
}




// <<<<<<<<<<<<<<<< OTROS >>>>>>>>>>>>>>>>
/************************************************************
 * ledOn:
 *		Enciende led con pin n (RED, YELLOW o GREEN)
 ***********************************************************/
void ledOn(int n) {
  digitalWrite(n, HIGH);
}

/************************************************************
 * ledOff:
 *		Apaga led con pin n (RED, YELLOW o GREEN)
 ***********************************************************/
void ledOff(int n) {
  digitalWrite(n, LOW);
}

/************************************************************
 * beep:
 *		Emite pitidito
 ***********************************************************/
void beep() {
  analogWrite(CLAX, HIGH);
  delay(100);
  analogWrite(CLAX, LOW);
}

/************************************************************
 * checkwall:
 *		Enciende led con pin n (RED, YELLOW o GREEN)
 ***********************************************************/
bool checkWall() {
  // Arrancamos sensor
  digitalWrite(OS,LOW);
  delayMicroseconds(5);
  digitalWrite(OS, HIGH);
  delayMicroseconds(10);

  // Calculamos distancia
  wall_time = pulseIn(IS, HIGH);
  float distancia = int(0.017 * wall_time);

  // Devolvemos si hay pared
  if(distancia < MAX_WALL_DISTANCE) {
    ledOn(RED);
    if (moving && wallStop) {
    	parar();
    	wallStop = false;
    }
    return true;
  } else {
    ledOff(RED);
    return false;
  }
}

/************************************************************
 * toRadianes:
 *		\param[in]	grados	· Grados
 *		\param[out]	float		· Radianes
 ***********************************************************/
float toRadianes(float grados) {
  return (grados * (M_PI / 180));
}

/************************************************************
 * toRadianes:
 *		\param[in]	radianes	· Radianes
 *		\param[out]	float			· Grados
 ***********************************************************/
float toGrados(float radianes) {
  return (radianes * (180 / M_PI));
}

/************************************************************
 * powerUp:
 *		Aumenta la potencia en 10
 ***********************************************************/
void powerUp() {
	if (power < 200) {
		power = power + 10;
	}
	control.print("  >> POWER: ");
	control.println(power);
}

/************************************************************
 * powerDown:
 *		Disminuye la potencia en 10
 ***********************************************************/
void powerDown() {
	if (power > 10) {
		power = power - 10;
	} else {
		power = 0;
		parar();
	}
	control.print("  >> POWER: ");
	control.println(power);
}

/************************************************************
 * clearBuffer:
 *		Limpia el buffer complex_cmd
 ***********************************************************/
void clearBuffer() {
	for(int i = 0; i < CMD_MAX_LENGTH; i++)
		complex_cmd[i] = ' ';
}

/************************************************************
 * ACCELEROMETER
 * parseOvalue:
 *		Actualiza power_left y power_right según el valor del móvil
 ***********************************************************/
void parseOvalue() {
	// Si el valor es negativo, gira a la izquierda
 	if (ovalue < 0) {
		power_left = 1 - ((10 + ovalue) / 10);
		power_right = 1;
	// Si el valor es positivo, gira a la derecha
	} else if (ovalue > 0) {
		power_left = 1;
		power_right = 1 - ((10 - ovalue) / 10);
	// Si el valor es 0 (complicado), ambos motores van a 100%
	} else {
		power_left = 1;
		power_right = 1;
  }
}

// CÓDIGO ÓSCAR <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void oscar() {
	stopMode = false;
	while (!stopMode) {
		while (control.available()) {
	    cmd = control.read();
	    if (cmd == '#') {
	    } else if (cmd == '+') {
	      if (velocidad < 241) {
	        velocidad = velocidad + 10;
	      }
	    } else if (cmd == 'm') {
		    if (velocidad > 29) {
		      velocidad = velocidad - 30;
		    } 
	    } else if (cmd == '$') {
	      posicion = 0;
	      valor = atof(buffer) + 0.8;
	      vaciarArray();
	      
	    } else if (cmd == 'x') {
	    	stopMode = true;
	    	auxMode = MODE_MANUAL;
	    } else {
	      buffer[posicion] = cmd;
	      posicion += 1;
	    }
		}
		analogWrite(A, giroIzquierda());
	  digitalWrite(A1, HIGH);
	  digitalWrite(A2, LOW);
	
	  analogWrite(B, giroDerecha());
	  digitalWrite(B1, HIGH);
	  digitalWrite(B2, LOW);
	}
	parar();
	mode = auxMode;
	auxMode = MODE_MANUAL;
	stopMode = false;
}

double giroIzquierda() {
  if (valor < 0) {
    if (valor < -10) {
      valor = -10;
    }
    double izq = velocidad + velocidad * (valor / 10);
    return izq;
  }  else {
    return velocidad;
  }
}

double giroDerecha() {
  Serial.println(valor);
  if (valor >= 0 ) {
    if (valor > 10) {
      valor = 10;
    }
    double der = velocidad - velocidad * (valor / 10);
    return der;
  } else {
    return velocidad;
  }
}

void vaciarArray() {
  int i;
  for (i = 0; i < 10; i++ ) {
    buffer[i] = ' ';
  }
}

// END    ÓSCAR >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/**
 * TODO:
 * 		· Cambiar coordenadas x e y por puntos
 * 		· Hacer pila de movimientos
 * 		· Hacer modo coordenadas
 */
