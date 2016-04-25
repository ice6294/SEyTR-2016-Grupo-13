/***********************************************
 *  ARDUINO         SEyTR                      *
 *                                             *
 *  Proyecto: RoboTracing           GRUPO   4  *
 *  Autores:                                   *
 *     Germán Alonso Azqutia        GIS + GII  *
 *     Oscar Ballesteros Izquierdo  GIS + GII  *
 *     Luis León Gámez              GIS + GII  *
 **********************************************/

 
// <<<<<<<<<<<<<<<< INCLUDES >>>>>>>>>>>>>>>>
#include <stdio.h>
#include <string.h>
#include <math.h>

// <<<<<<<<<<<<<<<< DEFINES >>>>>>>>>>>>>>>>
#define PI 3.14159265359
#define true 1
#define false 0
 typedef int bool;


 // <<<<<<<<<<<<<<<< DECLARATIONS >>>>>>>>>>>>>>>>
void move(float n);
void rotate(float degrees);
void calculateNewCoords();
float getAngleTo(float new_x, float new_y);
float getDiff(float new_x, float new_y);
void rotateTo(float dest);
bool inEnd(float end_x, float end_y);
void goTo(float end_x, float end_y);
void imprimir();
void imprimir1(float n);
void imprimir2(float n1, float n2);
void success();
void fail();
float toRadianes(float grados);
float toGrados(float radianes);
void cleanAll();
void scanRest();


// <<<<<<<<<<<<<<<< VARIABLES >>>>>>>>>>>>>>>>
float cy = 0;
float cx = 0;
float dist = 0;
float angle = 0;

// <<<<<<<<<<<<<<<< SETUP >>>>>>>>>>>>>>>>


// <<<<<<<<<<<<<<<< LOOP >>>>>>>>>>>>>>>>
int main(int argc, char **argv) {
	bool end = false;
	float n1;
	float n2;
	float n3;
	char c;
	while(!end) {
		printf("\n\n\t\t\t\t\t\t\tMENÚ\n\n");
		printf("[ w > Avanzar | s > Retroceder | a > Girar izq | d > Girar der | t > Angle To | g > Go To | r > Rotate To | l > atan2(y,x) ]\n");
		printf("[ q > Quit    | b > Restart    | c > Colocate  | i > Imprimir  | f > Imprimir All Info To | x > Get diff between 2 angles  ]\n");
		printf("Commando: ");
		scanf("%c", &c);
		scanRest();
		switch(c) {
			// QUIT
			case 'q':
				end = true;
				break;
			// AVANZAR
			case 'w':
				printf("Avanzar:");
				scanf("%f",&n1);
				scanRest();
				move(n1);
				imprimir();
				break;
			// RETROCEDER
			case 's':
				printf("Retroceder:");
				scanf("%f",&n1);
				scanRest();
				move(-n1);
				imprimir();
				break;
			// GIRAR IZQ
			case 'a':
				printf("Girar izq:");
				scanf("%f",&n1);
				scanRest();
				rotate(n1);
				imprimir();
				break;
			// GIRAR DER
			case 'd':
				printf("Girar der:");
				scanf("%f",&n1);
				scanRest();
				rotate(-n1);
				imprimir();
				break;
			// ANGLE TO
			case 't':
				printf("Angle to:\n");
				printf("   x:");
				scanf("%f",&n1);
				scanRest();
				printf("   y:");
				scanf("%f",&n2);
				scanRest();
				printf("\n    degrees: %fº\n",getAngleTo(n1,n2));
				//printf("\nPress enter:");
				//scanRest();
				break;
			// GO TO
			case 'g':
				printf("Go to:\n");
				printf("   x:");
				scanf("%f",&n1);
				scanRest();
				printf("   y:");
				scanf("%f",&n2);
				scanRest();
				goTo(n1,n2);
				imprimir();
				//printf("\nPress enter:");
				//scanRest();
				break;
			// RESTART
			case 'b':
				printf("    ALL VALUES RESTARTED TO 0\n\n");
				cleanAll();
				break;
			// COLOCATE
			case 'c':
				printf("Colocate\n");
				printf("   x:");
				scanf("%f",&n1);
				scanRest();
				cx = n1;
				printf("   y:");
				scanf("%f",&n2);
				scanRest();
				cy = n2;
				printf("   degrees:");
				scanf("%f",&n3);
				scanRest();
				angle = n3;
				break;
			// ALL INFO TO
			case 'f':
				printf("Get all to:\n");
				printf("   x:");
				scanf("%f",&n1);
				scanRest();
				printf("   y:");
				scanf("%f",&n2);
				scanRest();
				imprimir2(n1,n2);
				break;
			// ROTATE TO
			case 'r':
				printf("Angle to rotate:");
				scanf("%f",&n1);
				scanRest();
				rotateTo(n1);
				imprimir();
				break;
			// DEGREES BETWEEN ANGLES
			case 'x':
				printf("    Angle 1:");
				scanf("%f",&n1);
				scanRest();
				printf("    Angle 2:");
				scanf("%f",&n2);
				scanRest();
				printf("\nDegrees between both: %f\n", getDiff(n1,n2));
				break;
			// ARCOTANGENTE
			case 'l':
				printf("    x:");
				scanf("%f",&n1);
				scanRest();
				printf("    y:");
				scanf("%f",&n2);
				scanRest();
				printf("\n    atan: %f\n", toGrados(atan2(n2,n1)));
				break;
			// IMPRIMIR
			case 'i':
				imprimir();
				break;
			// DEFAULT
			default:
				printf("No entiendo :S\n");
				break;
		}
	}
	printf("Ciao! :)\n\n");
	return 0;
}


// <<<<<<<<<<<<<<<< COORDENADAS >>>>>>>>>>>>>>>>
void move(float n) {
	dist += n;
	calculateNewCoords();
}

// IZQ -> degrees +
// DER -> degrees -
void rotate(float degrees) {
	angle += degrees;
	while (angle > 360) {
		angle -= 360;
	}
	while (angle < 0) {
		angle += 360;
	}
}

void calculateNewCoords() {
	cx += dist * cos(toRadianes(angle));
	cy += dist * sin(toRadianes(angle));
	dist = 0;
}


// <<<<<<<<<<<<<<<< AUTO-CONTROL >>>>>>>>>>>>>>>>
float getAngleTo(float new_x, float new_y) {
	float diff_x = new_x - cx;
	float diff_y = new_y - cy;
	// Calculamos con la arcotange y los dos catetos el angulo hacia el origen
	float atanangle = toGrados(atan2(diff_y, diff_x));
	float angleToNewLocate;
	// Re-ajustamos el ángulo dependiendo del cuadrante (si está en el primero (+X,+Y) no es necesario)
	angleToNewLocate = atanangle;
	if(angleToNewLocate > 180) {
		// Lado (-X, _)
		angleToNewLocate = 360 + atanangle;
	}
	// Corregimos valores fuera de rango
	return angleToNewLocate;
}

// Giro a la izquierda -> POS +
// Giro a la derecha   -> NEG -
float getDiff(float angle1, float angle2) {
	float aux = angle2 - angle1;
	if (aux > 180) {
		aux -= 360;
	}
	if (aux < -180) {
		aux +=360;
	}
	return aux;
}

// dest: angulo destino
void rotateTo(float dest) {
	// Mientras no vire hacia el destino
	float diff = getDiff(angle, dest);
	bool exito = fabsf(diff) < 1;
	int max = 0;
	while(!exito) {
		printf("        . diff = %.2fº", diff);
		// Toma el camino más rápido para girar
		if(diff < 0) {
			printf(" -> der\n");
			rotate(-1); // Giramos hacia la derecha
		} else {
			printf(" -> izq\n");
			rotate(1); // Giramos hacia la izquierda
		}
		diff = getDiff(angle, dest);
		// Sale del bucle si está orientado hacia el destino
		exito = fabsf(diff) < 1;
	}
}

bool inEnd(float end_x, float end_y) {
	return ( cx < (end_x + 1)
		&& cx > (end_x - 1)
		&& cy < (end_y + 1)
		&& cy > (end_y - 1) );
}

void goTo(float end_x, float end_y) {
	// Vira hacia el punto origen (si no está ya orientado hacia él)
	float end_angle = getAngleTo(end_x, end_y);
	imprimir1(end_angle);
	// Mientras no haya llegado a casa
	bool exito = inEnd(end_x, end_y);
	int max = 0;
	while(!exito) {
		// Avanza hacia allí
		printf("    . angle  : %.2f\n", angle);
		end_angle = getAngleTo(end_x, end_y);
		printf("    . angleTo: %.2f\n", end_angle);
		rotateTo(end_angle);
		move(0.5);
		printf("    . avanza a [%.2f, %.2f]\n", cx, cy);
		// exito -> ha llegado
		exito = inEnd(end_x, end_y);
		// Max avances en esa dirección
		if (max > 1000 && !exito) {
			break;
		} else {
			max++;
		}
	}
	// Cuando haya llegado/lo hayan parado -> para
	if (exito) {
		success();
	} else {
		fail();
	}
}


// <<<<<<<<<<<<<<<< IMPRIMIR >>>>>>>>>>>>>>>>
void imprimir() {
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printf("  x: %f\n",cx);
	printf("  y: %f\n",cy);
	printf("  actual angle: %fº\n",angle);
	float aux = getAngleTo(0, 0);
	printf("  angle to zero [0, 0]: %fº\n", aux);
	printf("  diff: %fº\n", getDiff(angle,aux));
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

void imprimir1(float n) {
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printf("  x: %f\n",cx);
	printf("  y: %f\n",cy);
	printf("  actual angle: %fº\n",angle);
	printf("  diff to %f: %fº\n", n, getDiff(angle,n));
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

void imprimir2(float n1, float n2) {
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printf("  x: %f\n",cx);
	printf("  y: %f\n",cy);
	printf("  actual angle: %fº\n",angle);
	float aux = getAngleTo(n1, n2);
	printf("  angle to pos [%.0f, %.0f]: %fº\n", n1, n2, aux);
	printf("  diff: %fº\n", getDiff(angle,aux));
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

void success() {
	printf("#######################\n");
	printf("######## EXITO ########\n");
	printf("#######################\n\n");
}

void fail() {
	printf("#######################\n");
	printf("######### :( ##########\n");
	printf("#######################\n\n");
}


// <<<<<<<<<<<<<<<< OTROS >>>>>>>>>>>>>>>>
float toRadianes(float grados) {
	return (grados * (PI / 180));
}

float toGrados(float radianes) {
	return (radianes * (180 / PI));
}

void cleanAll() {
	cx = 0;
	cy = 0;
	dist = 0;
	angle = 0;
}

void scanRest() {
	while (getchar()!='\n');
	return;
}
